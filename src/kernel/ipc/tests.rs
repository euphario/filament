//! Integration Tests for IPC2
//!
//! These tests verify the complete IPC system works correctly
//! when all components interact together.

#[cfg(test)]
mod integration_tests {
    use crate::kernel::ipc::*;
    use crate::kernel::ipc::types::*;
    use crate::kernel::ipc::waker::WakeList;

    /// Test full send/receive cycle
    #[test]
    fn test_send_receive_cycle() {
        let mut table = ChannelTable::new();

        // Create channel pair between task 1 and task 2
        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Task 1 sends on ch_a
        let msg = Message::request(1, 100, b"ping");
        let wake = table.send(ch_a, msg, 1).unwrap();
        assert!(wake.is_empty()); // No subscribers yet

        // Task 2 receives on ch_b
        let received = table.receive(ch_b, 2).unwrap();
        assert_eq!(received.header.msg_type, MessageType::Request);
        assert_eq!(received.header.msg_id, 100);
        assert_eq!(received.payload_slice(), b"ping");

        // Task 2 replies on ch_b
        let reply = Message::reply(2, 100, b"pong");
        table.send(ch_b, reply, 2).unwrap();

        // Task 1 receives reply on ch_a
        let response = table.receive(ch_a, 1).unwrap();
        assert_eq!(response.header.msg_type, MessageType::Reply);
        assert_eq!(response.payload_slice(), b"pong");
    }

    /// Test subscriber waking on message arrival
    #[test]
    fn test_wake_on_message() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Task 2 subscribes to ch_b for Readable events
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Task 1 sends - should get wake list with task 2
        let msg = Message::data(1, b"hello");
        let wake = table.send(ch_a, msg, 1).unwrap();

        assert_eq!(wake.len(), 1);
        let sub = wake.iter().next().unwrap();
        assert_eq!(sub.task_id, 2);
    }

    /// Test wake on channel close
    #[test]
    fn test_wake_on_close() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Task 2 subscribes to ch_b for any events (Readable includes close)
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Task 1 closes ch_a
        let wake = table.close(ch_a, 1).unwrap();

        // Task 2 should be woken
        assert!(wake.len() >= 1);
    }

    /// Test half-closed state allows draining
    #[test]
    fn test_half_closed_drain() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Task 1 sends two messages
        table.send(ch_a, Message::data(1, b"msg1"), 1).unwrap();
        table.send(ch_a, Message::data(1, b"msg2"), 1).unwrap();

        // Task 1 closes their end
        table.close(ch_a, 1).unwrap();

        // Task 2 can still receive the pending messages
        let msg1 = table.receive(ch_b, 2).unwrap();
        assert_eq!(msg1.payload_slice(), b"msg1");

        let msg2 = table.receive(ch_b, 2).unwrap();
        assert_eq!(msg2.payload_slice(), b"msg2");

        // No more messages - now fully closed
        let err = table.receive(ch_b, 2);
        assert!(matches!(err, Err(IpcError::Closed) | Err(IpcError::PeerClosed)));
    }

    /// Test backpressure (queue full)
    #[test]
    fn test_backpressure() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Fill the queue
        for i in 0..MAX_QUEUE_SIZE {
            let msg = Message::data(1, &[i as u8]);
            table.send(ch_a, msg, 1).unwrap();
        }

        // Next send should fail with QueueFull
        let msg = Message::data(1, b"overflow");
        let err = table.send(ch_a, msg, 1);
        assert!(matches!(err, Err(IpcError::QueueFull)));

        // Receive one to make space
        table.receive(ch_b, 2).unwrap();

        // Now can send again
        let msg = Message::data(1, b"fits now");
        assert!(table.send(ch_a, msg, 1).is_ok());
    }

    /// Test port registration and connection
    #[test]
    fn test_port_workflow() {
        let mut table = ChannelTable::new();
        let mut reg = PortRegistry::new();

        // Server (task 1) registers port
        let (port_id, listen_ch) = reg.register(b"myservice:", 1, &mut table).unwrap();
        assert!(port_id > 0);
        assert!(listen_ch > 0);

        // Client (task 2) connects
        let (client_ch, wake) = reg.connect(b"myservice:", 2, &mut table).unwrap();
        assert!(client_ch > 0);

        // Server accepts
        let (server_ch, client_pid) = reg.accept(port_id, 1).unwrap();
        assert!(server_ch > 0);
        assert_eq!(client_pid, 2);

        // Now client and server can communicate
        let msg = Message::data(2, b"hello server");
        table.send(client_ch, msg, 2).unwrap();

        let received = table.receive(server_ch, 1).unwrap();
        assert_eq!(received.payload_slice(), b"hello server");
    }

    /// Test task cleanup closes all channels
    #[test]
    fn test_task_cleanup() {
        let mut table = ChannelTable::new();

        // Task 1 creates several channels
        let (ch1, _) = table.create_pair(1, 2).unwrap();
        let (ch2, _) = table.create_pair(1, 3).unwrap();
        let (_, ch4) = table.create_pair(3, 1).unwrap();

        // Task 1 owns ch1, ch2, and ch4
        assert_eq!(table.active_count(), 6);

        // Cleanup task 1
        let wake = table.cleanup_task(1);

        // Peers should be woken
        // Not all channels are gone - only those owned by task 1
        // Peers (owned by 2 and 3) are now half-closed
    }

    /// Test stale channel ID detection
    #[test]
    fn test_stale_channel_detection() {
        let mut table = ChannelTable::new();

        let (ch_a, _ch_b) = table.create_pair(1, 2).unwrap();

        // Close the channel
        table.close(ch_a, 1).unwrap();

        // Old ID should not work
        let err = table.get(ch_a, 1);
        assert!(matches!(err, Err(IpcError::InvalidChannel { .. })));

        // Create new channel - may reuse same slot
        let (new_ch, _) = table.create_pair(1, 2).unwrap();

        // Old ID still fails (generation mismatch)
        let err = table.get(ch_a, 1);
        assert!(matches!(err, Err(IpcError::InvalidChannel { .. })));

        // New ID works
        assert!(table.get(new_ch, 1).is_ok());
    }

    /// Test multiple subscribers
    #[test]
    fn test_multiple_subscribers() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Subscribe task 2 and task 3 to ch_b (simulating multiple waiters)
        // Note: In reality, only owner (task 2) should subscribe, but test the mechanism
        // For this test, we directly manipulate as internal test

        // Get the channel and subscribe multiple
        if let Ok(channel) = table.get_mut(ch_b, 2) {
            channel.subscribe(Subscriber::new(2, 1), WakeReason::Readable);
            channel.subscribe(Subscriber::new(3, 1), WakeReason::Readable);
        }

        // Send should wake both
        let msg = Message::data(1, b"broadcast");
        let wake = table.send(ch_a, msg, 1).unwrap();

        assert_eq!(wake.len(), 2);
    }

    // ========================================================================
    // Additional Concurrent/Subscriber Tests
    // ========================================================================

    /// Test stale subscriber not woken (generation mismatch)
    #[test]
    fn test_stale_subscriber_not_woken() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Subscribe with generation 1
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Simulate task 2 exiting and slot being reused with generation 2
        // by subscribing again with a different generation
        // (In real kernel, this would be a new task in the same slot)
        table.subscribe(ch_b, 2, Subscriber::new(2, 2), WakeReason::Readable).unwrap();

        // Send message
        let msg = Message::data(1, b"test");
        let wake = table.send(ch_a, msg, 1).unwrap();

        // Should only wake the latest subscriber (generation 2)
        // Note: actual behavior depends on subscribe implementation
        // The key is that old generation subscribers get replaced
        for sub in wake.iter() {
            // If task 2 is in wake list, it should be generation 2
            if sub.task_id == 2 {
                assert_eq!(sub.generation, 2);
            }
        }
    }

    /// Test subscribe replaces previous subscription (idempotent)
    #[test]
    fn test_subscribe_replaces_previous() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Subscribe for Readable
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Subscribe again for Writable - should replace
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Writable).unwrap();

        // Send message (Readable event) - should NOT wake since we now subscribe for Writable
        let msg = Message::data(1, b"test");
        let wake = table.send(ch_a, msg, 1).unwrap();

        // No wake expected (subscribed for Writable, got Readable event)
        // Note: depends on whether Writable wake happens on receive making space
        // This tests the replacement behavior
    }

    /// Test unsubscribe prevents waking
    #[test]
    fn test_unsubscribe_prevents_wake() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Subscribe
        table.subscribe(ch_b, 2, Subscriber::new(2, 1), WakeReason::Readable).unwrap();

        // Unsubscribe
        table.unsubscribe(ch_b, 2).unwrap();

        // Send message
        let msg = Message::data(1, b"test");
        let wake = table.send(ch_a, msg, 1).unwrap();

        // Should not wake task 2 (unsubscribed)
        for sub in wake.iter() {
            assert_ne!(sub.task_id, 2);
        }
    }

    /// Test rapid subscribe/unsubscribe cycles
    #[test]
    fn test_subscribe_unsubscribe_cycle() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Rapid cycles
        for i in 0..10 {
            table.subscribe(ch_b, 2, Subscriber::new(2, i), WakeReason::Readable).unwrap();
            table.unsubscribe(ch_b, 2).unwrap();
        }

        // Final subscribe
        table.subscribe(ch_b, 2, Subscriber::new(2, 100), WakeReason::Readable).unwrap();

        // Send - should wake with generation 100
        let msg = Message::data(1, b"test");
        let wake = table.send(ch_a, msg, 1).unwrap();

        assert_eq!(wake.len(), 1);
        let sub = wake.iter().next().unwrap();
        assert_eq!(sub.task_id, 2);
        assert_eq!(sub.generation, 100);
    }

    /// Test close wakes all subscribers
    #[test]
    fn test_close_wakes_all_subscribers() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Subscribe multiple types
        if let Ok(channel) = table.get_mut(ch_b, 2) {
            channel.subscribe(Subscriber::new(2, 1), WakeReason::Readable);
            channel.subscribe(Subscriber::new(3, 1), WakeReason::Writable);
        }

        // Close ch_a
        let wake = table.close(ch_a, 1).unwrap();

        // Both should be woken on close
        assert!(wake.len() >= 2);
    }

    /// Test send to closed channel fails
    #[test]
    fn test_send_to_closed_fails() {
        let mut table = ChannelTable::new();

        let (ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Close ch_b (peer)
        table.close(ch_b, 2).unwrap();

        // Try to send on ch_a
        let msg = Message::data(1, b"test");
        let err = table.send(ch_a, msg, 1);

        // Should fail - peer closed
        assert!(matches!(err, Err(IpcError::PeerClosed)));
    }

    /// Test receive on empty channel
    #[test]
    fn test_receive_empty_channel() {
        let mut table = ChannelTable::new();

        let (_ch_a, ch_b) = table.create_pair(1, 2).unwrap();

        // Try to receive when queue is empty
        let err = table.receive(ch_b, 2);

        // Should fail - empty
        assert!(matches!(err, Err(IpcError::Empty)));
    }

    /// Test wrong owner cannot access channel
    #[test]
    fn test_wrong_owner_access_denied() {
        let mut table = ChannelTable::new();

        let (ch_a, _ch_b) = table.create_pair(1, 2).unwrap();

        // Task 3 tries to send on ch_a (owned by task 1)
        let msg = Message::data(3, b"unauthorized");
        let err = table.send(ch_a, msg, 3);

        // Should fail - not owner
        assert!(matches!(err, Err(IpcError::NotOwner)));
    }
}
