//! IPC Backend Implementation
//!
//! Implements the `IpcBackend` trait by delegating to the existing
//! channel table and port registry. This provides a clean trait boundary
//! without changing the internal implementation.

use crate::kernel::traits::ipc::{
    IpcBackend, IpcError as TraitIpcError, ChannelState as TraitChannelState,
    PortState as TraitPortState, Message as TraitMessage, WakeList as TraitWakeList,
    ChannelId, PortId, TaskId,
};
use crate::kernel::traits::waker::{
    Subscriber as TraitSubscriber,
    WakeReason as TraitWakeReason,
};
use super::{
    with_channel_table, with_port_registry,
    error::IpcError,
    channel::ChannelState,
    port::PortState,
    types::Message,
    waker::WakeList,
    traits::{Subscriber as InternalSubscriber, WakeReason as InternalWakeReason},
};

/// Global IPC backend instance
///
/// This is a zero-sized type that implements `IpcBackend` by delegating
/// to the global channel table and port registry.
pub struct KernelIpcBackend;

impl KernelIpcBackend {
    /// Get the global IPC backend instance
    pub const fn new() -> Self {
        Self
    }
}

// ============================================================================
// Type Conversions
// ============================================================================

/// Convert internal IpcError to trait IpcError
fn convert_error(e: IpcError) -> TraitIpcError {
    match e {
        IpcError::InvalidChannel { id: _ } => TraitIpcError::NotFound,
        IpcError::Closed => TraitIpcError::PeerClosed,
        IpcError::PeerClosed => TraitIpcError::PeerClosed,
        IpcError::NotOwner { .. } => TraitIpcError::NotOwner,
        IpcError::QueueFull => TraitIpcError::QueueFull,
        IpcError::WouldBlock => TraitIpcError::WouldBlock,
        IpcError::MessageTooLarge { .. } => TraitIpcError::InvalidArg,
        IpcError::PortExists { .. } => TraitIpcError::InvalidArg,
        IpcError::PortNotFound => TraitIpcError::NotFound,
        IpcError::InvalidPort { .. } => TraitIpcError::NotFound,
        IpcError::PortNotOwner { .. } => TraitIpcError::NotOwner,
        IpcError::NoPending => TraitIpcError::NoPending,
        IpcError::PendingFull => TraitIpcError::NoSlots,
        IpcError::NoChannelSpace => TraitIpcError::NoSlots,
        IpcError::NoPortSpace => TraitIpcError::NoSlots,
        IpcError::SubscribersFull => TraitIpcError::SubscribersFull,
        IpcError::StaleHandle { .. } => TraitIpcError::NotFound,
        IpcError::InvalidHandle => TraitIpcError::NotFound,
        IpcError::NotSupported => TraitIpcError::InvalidArg,
        IpcError::Timeout => TraitIpcError::WouldBlock,
        IpcError::Interrupted => TraitIpcError::WouldBlock,
        IpcError::Busy => TraitIpcError::InvalidArg,
        IpcError::Internal => TraitIpcError::InvalidArg,
    }
}

/// Convert internal ChannelState to trait ChannelState
fn convert_channel_state(s: &ChannelState) -> TraitChannelState {
    match s {
        ChannelState::Open { peer_id, peer_owner } => {
            TraitChannelState::Open {
                peer_id: *peer_id,
                peer_owner: *peer_owner,
            }
        }
        ChannelState::HalfClosed { messages_remaining } => {
            TraitChannelState::HalfClosed {
                messages_remaining: *messages_remaining,
            }
        }
        ChannelState::Closed => TraitChannelState::Closed,
    }
}

/// Convert internal PortState to trait PortState
fn convert_port_state(s: &PortState) -> TraitPortState {
    match s {
        PortState::Listening { pending_count } => {
            TraitPortState::Listening {
                pending_count: *pending_count,
            }
        }
        PortState::Closed => TraitPortState::Closed,
    }
}

/// Convert internal Message to trait Message
fn convert_message(m: Message) -> TraitMessage {
    TraitMessage::new(m.header.sender, m.payload_slice())
}

/// Convert trait Message to internal Message
fn convert_message_to_internal(m: &TraitMessage) -> Message {
    Message::data(m.sender, m.data())
}

/// Convert internal WakeList to trait WakeList
fn convert_wake_list(list: WakeList) -> TraitWakeList {
    let mut result = TraitWakeList::new();
    for sub in list.iter() {
        // Convert internal subscriber to trait subscriber
        result.push(TraitSubscriber::new(sub.task_id, sub.generation));
    }
    result
}

/// Convert trait WakeReason to internal WakeReason
fn convert_wake_reason(reason: TraitWakeReason) -> InternalWakeReason {
    match reason {
        TraitWakeReason::Readable => InternalWakeReason::Readable,
        TraitWakeReason::Writable => InternalWakeReason::Writable,
        TraitWakeReason::Closed => InternalWakeReason::Closed,
        TraitWakeReason::Accepted => InternalWakeReason::Accepted,
        TraitWakeReason::Timeout => InternalWakeReason::Timeout,
        TraitWakeReason::Signal => InternalWakeReason::Signal,
        TraitWakeReason::ChildExit => InternalWakeReason::ChildExit,
    }
}

/// Convert trait Subscriber to internal Subscriber
fn convert_subscriber(sub: TraitSubscriber) -> InternalSubscriber {
    InternalSubscriber::new(sub.task_id, sub.generation)
}

// ============================================================================
// IpcBackend Implementation
// ============================================================================

impl IpcBackend for KernelIpcBackend {
    fn create_channel_pair(
        &self,
        owner_a: TaskId,
        owner_b: TaskId,
    ) -> Result<(ChannelId, ChannelId), TraitIpcError> {
        super::create_channel_pair(owner_a, owner_b).map_err(convert_error)
    }

    fn register_port(
        &self,
        name: &[u8],
        owner: TaskId,
    ) -> Result<(PortId, ChannelId), TraitIpcError> {
        super::port_register(name, owner).map_err(convert_error)
    }

    fn connect_to_port(
        &self,
        name: &[u8],
        client: TaskId,
    ) -> Result<(ChannelId, TraitWakeList), TraitIpcError> {
        super::port_connect(name, client)
            .map(|(id, list, _owner)| (id, convert_wake_list(list)))
            .map_err(convert_error)
    }

    fn accept_port(
        &self,
        port_id: PortId,
        owner: TaskId,
    ) -> Result<(ChannelId, TaskId), TraitIpcError> {
        super::port_accept(port_id, owner).map_err(convert_error)
    }

    fn close_port(
        &self,
        port_id: PortId,
        owner: TaskId,
    ) -> Result<TraitWakeList, TraitIpcError> {
        super::port_unregister(port_id, owner)
            .map(convert_wake_list)
            .map_err(convert_error)
    }

    fn port_state(&self, port_id: PortId) -> Option<TraitPortState> {
        with_port_registry(|reg| {
            reg.get(port_id).map(|p| convert_port_state(p.state()))
        })
    }

    fn send(
        &self,
        channel_id: ChannelId,
        msg: &TraitMessage,
        caller: TaskId,
    ) -> Result<TraitWakeList, TraitIpcError> {
        let internal_msg = convert_message_to_internal(msg);
        super::send(channel_id, internal_msg, caller)
            .map(convert_wake_list)
            .map_err(convert_error)
    }

    fn receive(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
    ) -> Result<TraitMessage, TraitIpcError> {
        super::receive(channel_id, caller)
            .map(convert_message)
            .map_err(convert_error)
    }

    fn close_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
    ) -> Result<TraitWakeList, TraitIpcError> {
        super::close_channel(channel_id, caller)
            .map(convert_wake_list)
            .map_err(convert_error)
    }

    fn subscribe_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
        sub: TraitSubscriber,
        reason: TraitWakeReason,
    ) -> Result<(), TraitIpcError> {
        super::subscribe(channel_id, caller, convert_subscriber(sub), convert_wake_reason(reason))
            .map_err(convert_error)
    }

    fn poll_channel(
        &self,
        channel_id: ChannelId,
        caller: TaskId,
        filter: TraitWakeReason,
    ) -> Result<bool, TraitIpcError> {
        with_channel_table(|table| {
            table.poll(channel_id, caller, convert_wake_reason(filter))
        }).map_err(convert_error)
    }

    fn channel_has_messages(&self, channel_id: ChannelId) -> bool {
        // Use module-level function that doesn't require ownership check
        super::channel_has_messages(channel_id)
    }

    fn channel_is_closed(&self, channel_id: ChannelId) -> bool {
        // Use module-level function that doesn't require ownership check
        super::channel_is_closed(channel_id)
    }

    fn channel_state(&self, channel_id: ChannelId) -> Option<TraitChannelState> {
        with_channel_table(|table| {
            table.get_unchecked(channel_id)
                .map(|ch| convert_channel_state(ch.state()))
        })
    }

    fn cleanup_task(&self, task_id: TaskId) -> TraitWakeList {
        convert_wake_list(super::cleanup_task(task_id))
    }

    fn remove_subscriber(&self, task_id: TaskId) {
        super::remove_subscriber_from_all(task_id);
    }
}

// ============================================================================
// Global Instance
// ============================================================================

/// Global IPC backend
pub static IPC_BACKEND: KernelIpcBackend = KernelIpcBackend::new();

/// Get a reference to the global IPC backend
pub fn ipc_backend() -> &'static dyn IpcBackend {
    &IPC_BACKEND
}
