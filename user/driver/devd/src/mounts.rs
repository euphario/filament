//! Mount Table for devd
//!
//! Tracks filesystem mount points. Drivers register mounts via
//! REGISTER_MOUNT query, clients resolve paths via RESOLVE_PATH.

pub const MAX_MOUNTS: usize = 16;

/// Transport type for a mount point.
#[derive(Clone, Copy)]
pub enum MountTransport {
    /// Channel-based: connect to port by name
    Port { name: [u8; 32], name_len: u8 },
    /// DataPort-based: map shmem and use rings
    DataPort { shmem_id: u32 },
}

/// A single mount table entry.
#[derive(Clone, Copy)]
pub struct MountEntry {
    pub prefix: [u8; 64],
    pub prefix_len: u8,
    pub transport: MountTransport,
    pub owner_pid: u32,
}

impl MountEntry {
    const fn empty() -> Self {
        Self {
            prefix: [0; 64],
            prefix_len: 0,
            transport: MountTransport::Port { name: [0; 32], name_len: 0 },
            owner_pid: 0,
        }
    }

    pub fn prefix_bytes(&self) -> &[u8] {
        &self.prefix[..self.prefix_len as usize]
    }
}

/// Mount table: stores active mounts, supports longest-prefix resolution.
pub struct MountTable {
    entries: [Option<MountEntry>; MAX_MOUNTS],
    count: usize,
}

impl MountTable {
    pub const fn new() -> Self {
        Self {
            entries: [const { None }; MAX_MOUNTS],
            count: 0,
        }
    }

    /// Register a new mount point. Rejects duplicate prefixes.
    pub fn mount(&mut self, prefix: &[u8], transport: MountTransport, owner_pid: u32) -> Result<(), MountError> {
        if prefix.is_empty() || prefix.len() > 64 {
            return Err(MountError::InvalidPrefix);
        }

        // Check for duplicate prefix
        for entry in self.entries.iter().flatten() {
            if entry.prefix_bytes() == prefix {
                return Err(MountError::Duplicate);
            }
        }

        // Find free slot
        let slot = self.entries.iter().position(|e| e.is_none())
            .ok_or(MountError::TableFull)?;

        let mut e = MountEntry::empty();
        e.prefix[..prefix.len()].copy_from_slice(prefix);
        e.prefix_len = prefix.len() as u8;
        e.transport = transport;
        e.owner_pid = owner_pid;

        self.entries[slot] = Some(e);
        self.count += 1;
        Ok(())
    }

    /// Unmount a prefix. Only the owner or PID 1 can unmount.
    pub fn unmount(&mut self, prefix: &[u8], caller_pid: u32) -> Result<(), MountError> {
        for i in 0..MAX_MOUNTS {
            if let Some(entry) = &self.entries[i] {
                if entry.prefix_bytes() == prefix {
                    if entry.owner_pid != caller_pid && caller_pid != 1 {
                        return Err(MountError::Permission);
                    }
                    self.entries[i] = None;
                    self.count -= 1;
                    return Ok(());
                }
            }
        }
        Err(MountError::NotFound)
    }

    /// Resolve a path to (transport, remaining_path). Longest prefix match.
    pub fn resolve<'a>(&self, path: &'a [u8]) -> Option<(&MountTransport, &'a [u8])> {
        let mut best: Option<(usize, usize)> = None; // (index, prefix_len)

        for i in 0..MAX_MOUNTS {
            if let Some(entry) = &self.entries[i] {
                let prefix = entry.prefix_bytes();
                if path.len() >= prefix.len() && &path[..prefix.len()] == prefix {
                    if best.map_or(true, |(_, len)| prefix.len() > len) {
                        best = Some((i, prefix.len()));
                    }
                }
            }
        }

        // Also try matching without trailing slash
        if best.is_none() {
            for i in 0..MAX_MOUNTS {
                if let Some(entry) = &self.entries[i] {
                    let prefix = entry.prefix_bytes();
                    if prefix.len() > 1 && prefix[prefix.len() - 1] == b'/' {
                        let prefix_no_slash = &prefix[..prefix.len() - 1];
                        if path == prefix_no_slash {
                            return self.entries[i].as_ref()
                                .map(|e| (&e.transport, &path[path.len()..]));
                        }
                    }
                }
            }
        }

        best.and_then(|(idx, prefix_len)| {
            self.entries[idx].as_ref()
                .map(|e| (&e.transport, &path[prefix_len..]))
        })
    }

    /// Iterate over active entries.
    pub fn for_each<F: FnMut(&MountEntry)>(&self, mut f: F) {
        for entry in self.entries.iter().flatten() {
            f(entry);
        }
    }

    /// Remove all mounts owned by a specific PID.
    pub fn remove_by_owner(&mut self, pid: u32) {
        for i in 0..MAX_MOUNTS {
            if let Some(entry) = &self.entries[i] {
                if entry.owner_pid == pid {
                    self.entries[i] = None;
                    self.count -= 1;
                }
            }
        }
    }

    pub fn count(&self) -> usize {
        self.count
    }
}

#[derive(Clone, Copy, Debug)]
pub enum MountError {
    InvalidPrefix,
    Duplicate,
    TableFull,
    NotFound,
    Permission,
}
