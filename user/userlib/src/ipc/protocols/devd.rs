//! devd Protocol - Device Tree Service
//!
//! Type-safe IPC protocol for communicating with devd (device supervisor).
//!
//! # Overview
//!
//! devd provides a central registry for hardware devices and system services.
//! It maintains a path-based device tree where every resource has a unique
//! path and a set of key-value properties.
//!
//! # Device Tree Structure
//!
//! ```text
//! /services/
//!     gpio                {port: "gpio", pid: "6"}
//!     pcie                {port: "pcie", pid: "5"}
//! /bus/
//!     pcie0/
//!         01:00.0         {vendor: "14c3", device: "7990", class: "network"}
//!     i2c2/
//!         pca9555@20/
//!             io0_0       {type: "gpio", dir: "in", val: "0"}
//!             usb_vbus    {type: "gpio", dir: "out", val: "1"}
//! /block/
//!     sda                 {type: "msc", size: "32GB", driver: "fatfs"}
//! ```
//!
//! # Operations
//!
//! | Operation | Description |
//! |-----------|-------------|
//! | `Register` | Add a node to the tree (or update if exists) |
//! | `Update` | Update an existing node's properties |
//! | `Query` | Find nodes matching a glob pattern |
//! | `Subscribe` | Watch for changes matching a pattern |
//! | `Unsubscribe` | Stop watching for changes |
//! | `Remove` | Remove a node from the tree |
//!
//! # Glob Patterns
//!
//! Queries and subscriptions support glob patterns:
//! - `*` matches any characters except `/`
//! - `**` matches any characters including `/`
//! - `?` matches a single character
//!
//! Examples:
//! - `/bus/i2c2/*` - All direct children of i2c2
//! - `/bus/**` - All nodes under /bus recursively
//! - `/services/gpio` - Exact match
//!
//! # Usage Examples
//!
//! ## Using DevdClient (Recommended)
//!
//! ```rust,ignore
//! use userlib::ipc::DevdClient;
//! use userlib::ipc::protocols::devd::PropertyList;
//!
//! // Connect to devd
//! let mut devd = DevdClient::connect()?;
//!
//! // Register a GPIO pin
//! let props = PropertyList::new()
//!     .add("type", "gpio")
//!     .add("dir", "out")
//!     .add("val", "1");
//! devd.register("/bus/i2c2/pca9555@20/usb_vbus", props)?;
//!
//! // Query all GPIO pins
//! let nodes = devd.query("/bus/i2c2/pca9555@20/*")?;
//! for node in nodes.iter() {
//!     println!("{}", node.path_str());
//! }
//!
//! // Update a pin's value
//! let props = PropertyList::new()
//!     .add("type", "gpio")
//!     .add("dir", "out")
//!     .add("val", "0");
//! devd.update("/bus/i2c2/pca9555@20/usb_vbus", props)?;
//! ```
//!
//! ## Subscribing to Changes
//!
//! ```rust,ignore
//! // Subscribe to GPIO changes
//! devd.subscribe("/bus/i2c2/**")?;
//!
//! // Events will be delivered to your channel when nodes change
//! // (Created, Updated, or Removed events)
//! ```
//!
//! # Wire Format
//!
//! All messages use little-endian byte order:
//!
//! Request: `[cmd:u8][path_len:u8][path:bytes][property_count:u8][properties...]`
//! Response: `[status:u8][payload...]`

use super::super::error::{IpcError, IpcResult};
use super::super::protocol::{Protocol, Message};

// =============================================================================
// Protocol Definition
// =============================================================================

/// devd service protocol
pub struct DevdProtocol;

impl Protocol for DevdProtocol {
    type Request = DevdRequest;
    type Response = DevdResponse;
    const PORT_NAME: &'static [u8] = b"devd";
}

// =============================================================================
// Constants
// =============================================================================

/// Maximum path length
pub const MAX_PATH: usize = 64;
/// Maximum property key length
pub const MAX_KEY: usize = 16;
/// Maximum property value length
pub const MAX_VALUE: usize = 32;
/// Maximum properties per node
pub const MAX_PROPERTIES: usize = 8;
/// Maximum nodes in a query response
pub const MAX_NODES: usize = 16;

// Command codes
const CMD_REGISTER: u8 = 1;
const CMD_UPDATE: u8 = 2;
const CMD_QUERY: u8 = 3;
const CMD_SUBSCRIBE: u8 = 4;
const CMD_UNSUBSCRIBE: u8 = 5;
const CMD_REMOVE: u8 = 6;

// Response codes
const RESP_OK: u8 = 0;
const RESP_ERROR: u8 = 1;
const RESP_NODES: u8 = 2;
const RESP_EVENT: u8 = 3;

// =============================================================================
// Property
// =============================================================================

/// A key-value property
#[derive(Clone, Copy)]
pub struct Property {
    pub key: [u8; MAX_KEY],
    pub key_len: u8,
    pub value: [u8; MAX_VALUE],
    pub value_len: u8,
}

impl Property {
    pub const fn empty() -> Self {
        Self {
            key: [0; MAX_KEY],
            key_len: 0,
            value: [0; MAX_VALUE],
            value_len: 0,
        }
    }

    /// Create a property from key and value slices
    pub fn new(key: &[u8], value: &[u8]) -> Self {
        let mut p = Self::empty();
        let klen = key.len().min(MAX_KEY);
        let vlen = value.len().min(MAX_VALUE);
        p.key[..klen].copy_from_slice(&key[..klen]);
        p.key_len = klen as u8;
        p.value[..vlen].copy_from_slice(&value[..vlen]);
        p.value_len = vlen as u8;
        p
    }

    /// Create from string slices (convenience)
    pub fn from_str(key: &str, value: &str) -> Self {
        Self::new(key.as_bytes(), value.as_bytes())
    }

    pub fn key_str(&self) -> &str {
        core::str::from_utf8(&self.key[..self.key_len as usize]).unwrap_or("")
    }

    pub fn value_str(&self) -> &str {
        core::str::from_utf8(&self.value[..self.value_len as usize]).unwrap_or("")
    }

    /// Check if this property has the given key
    pub fn is_key(&self, key: &str) -> bool {
        self.key_str() == key
    }
}

impl core::fmt::Debug for Property {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}={}", self.key_str(), self.value_str())
    }
}

// =============================================================================
// PropertyList - Helper for building property arrays
// =============================================================================

/// Builder for property lists
#[derive(Clone)]
pub struct PropertyList {
    pub props: [Property; MAX_PROPERTIES],
    pub count: u8,
}

impl PropertyList {
    pub const fn new() -> Self {
        Self {
            props: [Property::empty(); MAX_PROPERTIES],
            count: 0,
        }
    }

    /// Add a property (builder pattern)
    pub fn add(mut self, key: &str, value: &str) -> Self {
        if (self.count as usize) < MAX_PROPERTIES {
            self.props[self.count as usize] = Property::from_str(key, value);
            self.count += 1;
        }
        self
    }

    /// Add a property (mutable)
    pub fn push(&mut self, key: &str, value: &str) {
        if (self.count as usize) < MAX_PROPERTIES {
            self.props[self.count as usize] = Property::from_str(key, value);
            self.count += 1;
        }
    }

    /// Get property by key
    pub fn get(&self, key: &str) -> Option<&str> {
        for i in 0..self.count as usize {
            if self.props[i].is_key(key) {
                return Some(self.props[i].value_str());
            }
        }
        None
    }

    /// Iterate over properties
    pub fn iter(&self) -> impl Iterator<Item = &Property> {
        self.props[..self.count as usize].iter()
    }
}

impl Default for PropertyList {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Node - A device tree node
// =============================================================================

/// A node in the device tree
#[derive(Clone)]
pub struct Node {
    pub path: [u8; MAX_PATH],
    pub path_len: u8,
    pub properties: PropertyList,
}

impl Node {
    pub const fn empty() -> Self {
        Self {
            path: [0; MAX_PATH],
            path_len: 0,
            properties: PropertyList::new(),
        }
    }

    pub fn new(path: &str) -> Self {
        let mut n = Self::empty();
        let len = path.len().min(MAX_PATH);
        n.path[..len].copy_from_slice(&path.as_bytes()[..len]);
        n.path_len = len as u8;
        n
    }

    pub fn path_str(&self) -> &str {
        core::str::from_utf8(&self.path[..self.path_len as usize]).unwrap_or("")
    }

    /// Get a property value
    pub fn get(&self, key: &str) -> Option<&str> {
        self.properties.get(key)
    }
}

// =============================================================================
// Event Types
// =============================================================================

/// Event types for subscriptions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EventType {
    /// Node was created
    Created = 1,
    /// Node properties were updated
    Updated = 2,
    /// Node was removed
    Removed = 3,
}

impl From<u8> for EventType {
    fn from(v: u8) -> Self {
        match v {
            1 => EventType::Created,
            2 => EventType::Updated,
            3 => EventType::Removed,
            _ => EventType::Updated,
        }
    }
}

// =============================================================================
// Requests
// =============================================================================

/// All requests to devd
#[derive(Clone)]
pub enum DevdRequest {
    /// Register a new node in the device tree
    Register {
        path: [u8; MAX_PATH],
        path_len: u8,
        properties: PropertyList,
    },

    /// Update properties of an existing node
    Update {
        path: [u8; MAX_PATH],
        path_len: u8,
        properties: PropertyList,
    },

    /// Query nodes matching a glob pattern
    /// Examples: "/bus/i2c2/*", "/services/*", "/bus/*/pca9555@*/*"
    Query {
        pattern: [u8; MAX_PATH],
        pattern_len: u8,
    },

    /// Subscribe to changes matching a pattern
    Subscribe {
        pattern: [u8; MAX_PATH],
        pattern_len: u8,
    },

    /// Unsubscribe from a pattern
    Unsubscribe {
        pattern: [u8; MAX_PATH],
        pattern_len: u8,
    },

    /// Remove a node from the tree
    Remove {
        path: [u8; MAX_PATH],
        path_len: u8,
    },
}

impl DevdRequest {
    /// Helper to create a Register request
    pub fn register(path: &str, props: PropertyList) -> Self {
        let mut path_buf = [0u8; MAX_PATH];
        let len = path.len().min(MAX_PATH);
        path_buf[..len].copy_from_slice(&path.as_bytes()[..len]);
        DevdRequest::Register {
            path: path_buf,
            path_len: len as u8,
            properties: props,
        }
    }

    /// Helper to create an Update request
    pub fn update(path: &str, props: PropertyList) -> Self {
        let mut path_buf = [0u8; MAX_PATH];
        let len = path.len().min(MAX_PATH);
        path_buf[..len].copy_from_slice(&path.as_bytes()[..len]);
        DevdRequest::Update {
            path: path_buf,
            path_len: len as u8,
            properties: props,
        }
    }

    /// Helper to create a Query request
    pub fn query(pattern: &str) -> Self {
        let mut pat_buf = [0u8; MAX_PATH];
        let len = pattern.len().min(MAX_PATH);
        pat_buf[..len].copy_from_slice(&pattern.as_bytes()[..len]);
        DevdRequest::Query {
            pattern: pat_buf,
            pattern_len: len as u8,
        }
    }

    /// Helper to create a Subscribe request
    pub fn subscribe(pattern: &str) -> Self {
        let mut pat_buf = [0u8; MAX_PATH];
        let len = pattern.len().min(MAX_PATH);
        pat_buf[..len].copy_from_slice(&pattern.as_bytes()[..len]);
        DevdRequest::Subscribe {
            pattern: pat_buf,
            pattern_len: len as u8,
        }
    }

    /// Helper to create a Remove request
    pub fn remove(path: &str) -> Self {
        let mut path_buf = [0u8; MAX_PATH];
        let len = path.len().min(MAX_PATH);
        path_buf[..len].copy_from_slice(&path.as_bytes()[..len]);
        DevdRequest::Remove {
            path: path_buf,
            path_len: len as u8,
        }
    }
}

// Property serialization size: key_len(1) + key(16) + value_len(1) + value(32) = 50
const PROP_SIZE: usize = 1 + MAX_KEY + 1 + MAX_VALUE;

fn serialize_properties(buf: &mut [u8], props: &PropertyList) -> usize {
    let mut offset = 0;
    buf[offset] = props.count;
    offset += 1;

    for i in 0..props.count as usize {
        let p = &props.props[i];
        buf[offset] = p.key_len;
        offset += 1;
        buf[offset..offset + MAX_KEY].copy_from_slice(&p.key);
        offset += MAX_KEY;
        buf[offset] = p.value_len;
        offset += 1;
        buf[offset..offset + MAX_VALUE].copy_from_slice(&p.value);
        offset += MAX_VALUE;
    }
    offset
}

fn deserialize_properties(buf: &[u8]) -> IpcResult<(PropertyList, usize)> {
    if buf.is_empty() {
        return Err(IpcError::Truncated);
    }

    let count = buf[0].min(MAX_PROPERTIES as u8);
    let mut offset = 1;
    let mut props = PropertyList::new();
    props.count = count;

    for i in 0..count as usize {
        if offset + PROP_SIZE > buf.len() {
            return Err(IpcError::Truncated);
        }
        props.props[i].key_len = buf[offset];
        offset += 1;
        props.props[i].key.copy_from_slice(&buf[offset..offset + MAX_KEY]);
        offset += MAX_KEY;
        props.props[i].value_len = buf[offset];
        offset += 1;
        props.props[i].value.copy_from_slice(&buf[offset..offset + MAX_VALUE]);
        offset += MAX_VALUE;
    }

    Ok((props, offset))
}

impl Message for DevdRequest {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            DevdRequest::Register { path, path_len, properties } => {
                // cmd(1) + path_len(1) + path(64) + properties
                let header = 1 + 1 + MAX_PATH;
                let props_size = 1 + (properties.count as usize * PROP_SIZE);
                if buf.len() < header + props_size {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_REGISTER;
                buf[1] = *path_len;
                buf[2..2 + MAX_PATH].copy_from_slice(path);
                let offset = 2 + MAX_PATH;
                let props_written = serialize_properties(&mut buf[offset..], properties);
                Ok(offset + props_written)
            }

            DevdRequest::Update { path, path_len, properties } => {
                let header = 1 + 1 + MAX_PATH;
                let props_size = 1 + (properties.count as usize * PROP_SIZE);
                if buf.len() < header + props_size {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_UPDATE;
                buf[1] = *path_len;
                buf[2..2 + MAX_PATH].copy_from_slice(path);
                let offset = 2 + MAX_PATH;
                let props_written = serialize_properties(&mut buf[offset..], properties);
                Ok(offset + props_written)
            }

            DevdRequest::Query { pattern, pattern_len } => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_QUERY;
                buf[1] = *pattern_len;
                buf[2..2 + MAX_PATH].copy_from_slice(pattern);
                Ok(2 + MAX_PATH)
            }

            DevdRequest::Subscribe { pattern, pattern_len } => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_SUBSCRIBE;
                buf[1] = *pattern_len;
                buf[2..2 + MAX_PATH].copy_from_slice(pattern);
                Ok(2 + MAX_PATH)
            }

            DevdRequest::Unsubscribe { pattern, pattern_len } => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_UNSUBSCRIBE;
                buf[1] = *pattern_len;
                buf[2..2 + MAX_PATH].copy_from_slice(pattern);
                Ok(2 + MAX_PATH)
            }

            DevdRequest::Remove { path, path_len } => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = CMD_REMOVE;
                buf[1] = *path_len;
                buf[2..2 + MAX_PATH].copy_from_slice(path);
                Ok(2 + MAX_PATH)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            CMD_REGISTER => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let path_len = buf[1];
                let mut path = [0u8; MAX_PATH];
                path.copy_from_slice(&buf[2..2 + MAX_PATH]);
                let (properties, props_len) = deserialize_properties(&buf[2 + MAX_PATH..])?;
                Ok((DevdRequest::Register { path, path_len, properties }, 2 + MAX_PATH + props_len))
            }

            CMD_UPDATE => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let path_len = buf[1];
                let mut path = [0u8; MAX_PATH];
                path.copy_from_slice(&buf[2..2 + MAX_PATH]);
                let (properties, props_len) = deserialize_properties(&buf[2 + MAX_PATH..])?;
                Ok((DevdRequest::Update { path, path_len, properties }, 2 + MAX_PATH + props_len))
            }

            CMD_QUERY => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let pattern_len = buf[1];
                let mut pattern = [0u8; MAX_PATH];
                pattern.copy_from_slice(&buf[2..2 + MAX_PATH]);
                Ok((DevdRequest::Query { pattern, pattern_len }, 2 + MAX_PATH))
            }

            CMD_SUBSCRIBE => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let pattern_len = buf[1];
                let mut pattern = [0u8; MAX_PATH];
                pattern.copy_from_slice(&buf[2..2 + MAX_PATH]);
                Ok((DevdRequest::Subscribe { pattern, pattern_len }, 2 + MAX_PATH))
            }

            CMD_UNSUBSCRIBE => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let pattern_len = buf[1];
                let mut pattern = [0u8; MAX_PATH];
                pattern.copy_from_slice(&buf[2..2 + MAX_PATH]);
                Ok((DevdRequest::Unsubscribe { pattern, pattern_len }, 2 + MAX_PATH))
            }

            CMD_REMOVE => {
                if buf.len() < 2 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let path_len = buf[1];
                let mut path = [0u8; MAX_PATH];
                path.copy_from_slice(&buf[2..2 + MAX_PATH]);
                Ok((DevdRequest::Remove { path, path_len }, 2 + MAX_PATH))
            }

            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            DevdRequest::Register { properties, .. } |
            DevdRequest::Update { properties, .. } => {
                2 + MAX_PATH + 1 + (properties.count as usize * PROP_SIZE)
            }
            DevdRequest::Query { .. } |
            DevdRequest::Subscribe { .. } |
            DevdRequest::Unsubscribe { .. } |
            DevdRequest::Remove { .. } => 2 + MAX_PATH,
        }
    }
}

// =============================================================================
// Responses
// =============================================================================

/// Node list for query responses
#[derive(Clone)]
pub struct NodeList {
    pub nodes: [Node; MAX_NODES],
    pub count: u8,
}

impl NodeList {
    pub const fn new() -> Self {
        const EMPTY: Node = Node::empty();
        Self {
            nodes: [EMPTY; MAX_NODES],
            count: 0,
        }
    }

    pub fn push(&mut self, node: Node) {
        if (self.count as usize) < MAX_NODES {
            self.nodes[self.count as usize] = node;
            self.count += 1;
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &Node> {
        self.nodes[..self.count as usize].iter()
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn len(&self) -> usize {
        self.count as usize
    }
}

impl Default for NodeList {
    fn default() -> Self {
        Self::new()
    }
}

/// All responses from devd
#[derive(Clone)]
pub enum DevdResponse {
    /// Success
    Ok,

    /// Error with code
    Error(i32),

    /// Query result - list of matching nodes
    Nodes(NodeList),

    /// Event notification (pushed to subscribers)
    Event {
        event_type: EventType,
        node: Node,
    },
}

impl Message for DevdResponse {
    fn serialize(&self, buf: &mut [u8]) -> IpcResult<usize> {
        match self {
            DevdResponse::Ok => {
                if buf.is_empty() {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = RESP_OK;
                Ok(1)
            }

            DevdResponse::Error(code) => {
                if buf.len() < 5 {
                    return Err(IpcError::MessageTooLarge);
                }
                buf[0] = RESP_ERROR;
                buf[1..5].copy_from_slice(&code.to_le_bytes());
                Ok(5)
            }

            DevdResponse::Nodes(list) => {
                // resp(1) + count(1) + nodes...
                // Each node: path_len(1) + path(64) + properties
                let mut offset = 0;
                buf[offset] = RESP_NODES;
                offset += 1;
                buf[offset] = list.count;
                offset += 1;

                for i in 0..list.count as usize {
                    let node = &list.nodes[i];
                    buf[offset] = node.path_len;
                    offset += 1;
                    buf[offset..offset + MAX_PATH].copy_from_slice(&node.path);
                    offset += MAX_PATH;
                    offset += serialize_properties(&mut buf[offset..], &node.properties);
                }

                Ok(offset)
            }

            DevdResponse::Event { event_type, node } => {
                // resp(1) + event_type(1) + path_len(1) + path(64) + properties
                let mut offset = 0;
                buf[offset] = RESP_EVENT;
                offset += 1;
                buf[offset] = *event_type as u8;
                offset += 1;
                buf[offset] = node.path_len;
                offset += 1;
                buf[offset..offset + MAX_PATH].copy_from_slice(&node.path);
                offset += MAX_PATH;
                offset += serialize_properties(&mut buf[offset..], &node.properties);
                Ok(offset)
            }
        }
    }

    fn deserialize(buf: &[u8]) -> IpcResult<(Self, usize)> {
        if buf.is_empty() {
            return Err(IpcError::Truncated);
        }

        match buf[0] {
            RESP_OK => Ok((DevdResponse::Ok, 1)),

            RESP_ERROR => {
                if buf.len() < 5 {
                    return Err(IpcError::Truncated);
                }
                let code = i32::from_le_bytes([buf[1], buf[2], buf[3], buf[4]]);
                Ok((DevdResponse::Error(code), 5))
            }

            RESP_NODES => {
                if buf.len() < 2 {
                    return Err(IpcError::Truncated);
                }
                let count = buf[1].min(MAX_NODES as u8);
                let mut list = NodeList::new();
                list.count = count;

                let mut offset = 2;
                for i in 0..count as usize {
                    if offset + 1 + MAX_PATH > buf.len() {
                        return Err(IpcError::Truncated);
                    }
                    list.nodes[i].path_len = buf[offset];
                    offset += 1;
                    list.nodes[i].path.copy_from_slice(&buf[offset..offset + MAX_PATH]);
                    offset += MAX_PATH;
                    let (props, props_len) = deserialize_properties(&buf[offset..])?;
                    list.nodes[i].properties = props;
                    offset += props_len;
                }

                Ok((DevdResponse::Nodes(list), offset))
            }

            RESP_EVENT => {
                if buf.len() < 3 + MAX_PATH {
                    return Err(IpcError::Truncated);
                }
                let event_type = EventType::from(buf[1]);
                let path_len = buf[2];
                let mut path = [0u8; MAX_PATH];
                path.copy_from_slice(&buf[3..3 + MAX_PATH]);
                let (properties, props_len) = deserialize_properties(&buf[3 + MAX_PATH..])?;

                let mut node = Node::empty();
                node.path = path;
                node.path_len = path_len;
                node.properties = properties;

                Ok((DevdResponse::Event { event_type, node }, 3 + MAX_PATH + props_len))
            }

            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    fn serialized_size(&self) -> usize {
        match self {
            DevdResponse::Ok => 1,
            DevdResponse::Error(_) => 5,
            DevdResponse::Nodes(list) => {
                let mut size = 2; // resp + count
                for i in 0..list.count as usize {
                    size += 1 + MAX_PATH + 1 + (list.nodes[i].properties.count as usize * PROP_SIZE);
                }
                size
            }
            DevdResponse::Event { node, .. } => {
                3 + MAX_PATH + 1 + (node.properties.count as usize * PROP_SIZE)
            }
        }
    }
}

// =============================================================================
// DevdClient - Convenience wrapper
// =============================================================================

/// Client for interacting with devd
pub struct DevdClient {
    inner: super::super::Client<DevdProtocol>,
}

impl DevdClient {
    /// Connect to devd
    pub fn connect() -> IpcResult<Self> {
        let inner = super::super::Client::<DevdProtocol>::connect()?;
        Ok(Self { inner })
    }

    /// Register a node in the device tree
    pub fn register(&mut self, path: &str, props: PropertyList) -> IpcResult<()> {
        let request = DevdRequest::register(path, props);
        let response = self.inner.request(&request)?;
        match response {
            DevdResponse::Ok => Ok(()),
            DevdResponse::Error(e) => Err(IpcError::ServerError(e)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Update a node's properties
    pub fn update(&mut self, path: &str, props: PropertyList) -> IpcResult<()> {
        let request = DevdRequest::update(path, props);
        let response = self.inner.request(&request)?;
        match response {
            DevdResponse::Ok => Ok(()),
            DevdResponse::Error(e) => Err(IpcError::ServerError(e)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Query nodes matching a pattern
    pub fn query(&mut self, pattern: &str) -> IpcResult<NodeList> {
        let request = DevdRequest::query(pattern);
        let response = self.inner.request(&request)?;
        match response {
            DevdResponse::Nodes(list) => Ok(list),
            DevdResponse::Error(e) => Err(IpcError::ServerError(e)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Subscribe to changes matching a pattern
    pub fn subscribe(&mut self, pattern: &str) -> IpcResult<()> {
        let request = DevdRequest::subscribe(pattern);
        let response = self.inner.request(&request)?;
        match response {
            DevdResponse::Ok => Ok(()),
            DevdResponse::Error(e) => Err(IpcError::ServerError(e)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }

    /// Remove a node
    pub fn remove(&mut self, path: &str) -> IpcResult<()> {
        let request = DevdRequest::remove(path);
        let response = self.inner.request(&request)?;
        match response {
            DevdResponse::Ok => Ok(()),
            DevdResponse::Error(e) => Err(IpcError::ServerError(e)),
            _ => Err(IpcError::UnexpectedMessage),
        }
    }
}
