//! Message Queue
//!
//! A fixed-size ring buffer for IPC messages. Each channel endpoint
//! has one queue for incoming messages.
//!
//! # Design
//!
//! - Fixed capacity (MAX_QUEUE_SIZE)
//! - Ring buffer with head/tail pointers
//! - No allocation, all inline
//! - Thread-safe when protected by channel table lock
//!
//! # Invariants
//!
//! 1. `count <= MAX_QUEUE_SIZE`
//! 2. `head` and `tail` are always `< MAX_QUEUE_SIZE`
//! 3. Empty: `count == 0`
//! 4. Full: `count == MAX_QUEUE_SIZE`

use super::types::{Message, MAX_QUEUE_SIZE};

/// Queue status for backpressure monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QueueStatus {
    /// Current number of messages in queue
    pub count: usize,
    /// Maximum queue capacity
    pub capacity: usize,
    /// True if queue is full
    pub full: bool,
    /// True if queue is empty
    pub empty: bool,
}

/// Message queue (ring buffer)
///
/// Messages are pushed at `tail` and popped from `head`.
/// The queue wraps around when reaching the end of the array.
pub struct MessageQueue {
    /// Message storage
    messages: [Message; MAX_QUEUE_SIZE],
    /// Index to pop from (oldest message)
    head: usize,
    /// Index to push to (next free slot)
    tail: usize,
    /// Number of messages currently in queue
    count: usize,
}

impl MessageQueue {
    /// Create a new empty queue
    pub const fn new() -> Self {
        const EMPTY_MSG: Message = Message::new();
        Self {
            messages: [EMPTY_MSG; MAX_QUEUE_SIZE],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// Check if queue is empty
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if queue is full
    #[inline]
    pub fn is_full(&self) -> bool {
        self.count >= MAX_QUEUE_SIZE
    }

    /// Get number of messages in queue
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Get available space in queue
    #[inline]
    pub fn available(&self) -> usize {
        MAX_QUEUE_SIZE - self.count
    }

    /// Push a message to the queue
    ///
    /// Returns `true` if message was queued, `false` if queue is full.
    pub fn push(&mut self, msg: Message) -> bool {
        if self.is_full() {
            return false;
        }

        self.messages[self.tail] = msg;
        self.tail = (self.tail + 1) % MAX_QUEUE_SIZE;
        self.count += 1;
        true
    }

    /// Pop a message from the queue
    ///
    /// Returns `Some(message)` if queue had a message, `None` if empty.
    pub fn pop(&mut self) -> Option<Message> {
        if self.is_empty() {
            return None;
        }

        let msg = self.messages[self.head];
        self.head = (self.head + 1) % MAX_QUEUE_SIZE;
        self.count -= 1;
        Some(msg)
    }

    /// Peek at the front message without removing
    pub fn peek(&self) -> Option<&Message> {
        if self.is_empty() {
            None
        } else {
            Some(&self.messages[self.head])
        }
    }

    /// Peek at the front message mutably without removing
    pub fn peek_mut(&mut self) -> Option<&mut Message> {
        if self.is_empty() {
            None
        } else {
            Some(&mut self.messages[self.head])
        }
    }

    /// Get queue status for backpressure monitoring
    pub fn status(&self) -> QueueStatus {
        QueueStatus {
            count: self.count,
            capacity: MAX_QUEUE_SIZE,
            full: self.is_full(),
            empty: self.is_empty(),
        }
    }

    /// Clear all messages from the queue
    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.count = 0;
    }

    /// Iterate over messages in queue (oldest first)
    ///
    /// This is a non-destructive iteration - messages stay in queue.
    pub fn iter(&self) -> QueueIter<'_> {
        QueueIter {
            queue: self,
            pos: 0,
        }
    }
}

impl Default for MessageQueue {
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator over queue messages
pub struct QueueIter<'a> {
    queue: &'a MessageQueue,
    pos: usize,
}

impl<'a> Iterator for QueueIter<'a> {
    type Item = &'a Message;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.queue.count {
            return None;
        }

        let idx = (self.queue.head + self.pos) % MAX_QUEUE_SIZE;
        self.pos += 1;
        Some(&self.queue.messages[idx])
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.queue.count - self.pos;
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for QueueIter<'a> {}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::ipc::types::MessageType;

    #[test]
    fn test_queue_new() {
        let q = MessageQueue::new();
        assert!(q.is_empty());
        assert!(!q.is_full());
        assert_eq!(q.len(), 0);
        assert_eq!(q.available(), MAX_QUEUE_SIZE);
    }

    #[test]
    fn test_queue_push_pop() {
        let mut q = MessageQueue::new();

        let msg = Message::data(1, b"hello");
        assert!(q.push(msg));
        assert!(!q.is_empty());
        assert_eq!(q.len(), 1);

        let popped = q.pop().unwrap();
        assert_eq!(popped.header.sender, 1);
        assert_eq!(popped.payload_slice(), b"hello");
        assert!(q.is_empty());
    }

    #[test]
    fn test_queue_full() {
        let mut q = MessageQueue::new();

        // Fill the queue
        for i in 0..MAX_QUEUE_SIZE {
            let msg = Message::data(i as u32, &[i as u8]);
            assert!(q.push(msg), "push {} should succeed", i);
        }

        assert!(q.is_full());
        assert_eq!(q.len(), MAX_QUEUE_SIZE);
        assert_eq!(q.available(), 0);

        // Try to push one more - should fail
        let extra = Message::data(99, b"extra");
        assert!(!q.push(extra));
    }

    #[test]
    fn test_queue_empty() {
        let mut q = MessageQueue::new();

        assert!(q.pop().is_none());
        assert!(q.peek().is_none());
    }

    #[test]
    fn test_queue_wrap_around() {
        let mut q = MessageQueue::new();

        // Push and pop a few times to move head/tail
        for round in 0..3 {
            // Fill queue
            for i in 0..MAX_QUEUE_SIZE {
                let msg = Message::data(i as u32, &[(round * 10 + i) as u8]);
                assert!(q.push(msg));
            }

            // Empty queue
            for i in 0..MAX_QUEUE_SIZE {
                let msg = q.pop().unwrap();
                assert_eq!(msg.payload_slice()[0], (round * 10 + i) as u8);
            }

            assert!(q.is_empty());
        }
    }

    #[test]
    fn test_queue_peek() {
        let mut q = MessageQueue::new();

        let msg = Message::data(42, b"peek test");
        q.push(msg);

        // Peek should not remove
        assert_eq!(q.peek().unwrap().header.sender, 42);
        assert_eq!(q.len(), 1);

        // Peek again
        assert_eq!(q.peek().unwrap().header.sender, 42);
        assert_eq!(q.len(), 1);

        // Pop should remove
        let popped = q.pop().unwrap();
        assert_eq!(popped.header.sender, 42);
        assert!(q.is_empty());
    }

    #[test]
    fn test_queue_status() {
        let mut q = MessageQueue::new();

        let status = q.status();
        assert!(status.empty);
        assert!(!status.full);
        assert_eq!(status.count, 0);
        assert_eq!(status.capacity, MAX_QUEUE_SIZE);

        // Add one message
        q.push(Message::new());
        let status = q.status();
        assert!(!status.empty);
        assert!(!status.full);
        assert_eq!(status.count, 1);
    }

    #[test]
    fn test_queue_clear() {
        let mut q = MessageQueue::new();

        for i in 0..5 {
            q.push(Message::data(i, b"test"));
        }

        assert_eq!(q.len(), 5);

        q.clear();
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn test_queue_iter() {
        let mut q = MessageQueue::new();

        for i in 0..5 {
            q.push(Message::data(i, &[i as u8]));
        }

        // Iterate and check order
        let senders: Vec<u32> = q.iter().map(|m| m.header.sender).collect();
        assert_eq!(senders, vec![0, 1, 2, 3, 4]);

        // Queue should be unchanged
        assert_eq!(q.len(), 5);
    }

    #[test]
    fn test_queue_iter_after_wrap() {
        let mut q = MessageQueue::new();

        // Push and pop to wrap around
        for _ in 0..3 {
            q.push(Message::data(0, b"temp"));
            q.pop();
        }

        // Now push new messages
        for i in 10..15 {
            q.push(Message::data(i, &[i as u8]));
        }

        let senders: Vec<u32> = q.iter().map(|m| m.header.sender).collect();
        assert_eq!(senders, vec![10, 11, 12, 13, 14]);
    }

    #[test]
    fn test_queue_fifo_order() {
        let mut q = MessageQueue::new();

        // Push messages 1, 2, 3
        q.push(Message::data(1, b"first"));
        q.push(Message::data(2, b"second"));
        q.push(Message::data(3, b"third"));

        // Should pop in same order
        assert_eq!(q.pop().unwrap().header.sender, 1);
        assert_eq!(q.pop().unwrap().header.sender, 2);
        assert_eq!(q.pop().unwrap().header.sender, 3);
    }
}
