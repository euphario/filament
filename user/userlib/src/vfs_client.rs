//! VFS Client - Reusable IPC wrapper for VFS operations
//!
//! Encapsulates the common pattern of connecting to vfsd and making requests.
//! Used by shell builtins and any other programs that need VFS access.

use crate::ipc::Channel;
use crate::vfs::{
    VfsHeader, msg, error, VfsError,
    ListDir, ReadFile, Stat, MakeDir, Remove, WriteFile,
    remove_flags, write_flags,
    DirEntries, DirEntry, FileData, FileInfo,
    Result as VfsResult,
};

/// VFS client for making filesystem requests
pub struct VfsClient {
    channel: Channel,
    seq_id: u32,
}

/// Result of a directory listing
pub struct DirListing<'a> {
    pub entries: &'a [u8],
    pub count: u16,
    pub has_more: bool,
}

/// Result of reading a file
pub struct FileContent<'a> {
    pub data: &'a [u8],
    pub eof: bool,
}

/// File information
pub struct FileStat {
    pub file_type: u8,
    pub size: u64,
}

impl VfsClient {
    /// Connect to vfsd
    pub fn connect() -> Result<Self, VfsError> {
        let channel = Channel::connect(b"vfs:")
            .map_err(|_| VfsError::IoError)?;
        Ok(Self { channel, seq_id: 0 })
    }

    /// Get next sequence ID
    fn next_seq_id(&mut self) -> u32 {
        self.seq_id = self.seq_id.wrapping_add(1);
        self.seq_id
    }

    /// List directory contents
    ///
    /// Returns raw bytes containing DirEntries header + entries.
    /// Use DirEntry::from_bytes to parse individual entries.
    pub fn list_dir<'a>(&mut self, path: &[u8], resp_buf: &'a mut [u8]) -> Result<DirListing<'a>, VfsError> {
        let seq_id = self.next_seq_id();
        let req = ListDir::new(seq_id);

        let mut req_buf = [0u8; 256];
        let len = req.write_to(&mut req_buf, path)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let resp_len = self.channel.recv(resp_buf)
            .map_err(|_| VfsError::IoError)?;

        // Parse header
        let header = VfsHeader::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        // Check for error
        if header.msg_type == msg::RESULT {
            let code = self.parse_result_code(&resp_buf[..resp_len]);
            return Err(VfsError::from_i32(code));
        }

        // Parse DIR_ENTRIES
        if header.msg_type != msg::DIR_ENTRIES {
            return Err(VfsError::InvalidMessage);
        }

        let entries = DirEntries::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        Ok(DirListing {
            entries: &resp_buf[..resp_len],
            count: entries.count,
            has_more: entries.more != 0,
        })
    }

    /// Read file contents
    pub fn read_file<'a>(
        &mut self,
        path: &[u8],
        offset: u64,
        max_len: u32,
        resp_buf: &'a mut [u8],
    ) -> Result<FileContent<'a>, VfsError> {
        let seq_id = self.next_seq_id();
        let req = ReadFile::new(seq_id, offset, max_len);

        let mut req_buf = [0u8; 256];
        let len = req.write_to(&mut req_buf, path)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let resp_len = self.channel.recv(resp_buf)
            .map_err(|_| VfsError::IoError)?;

        // Parse header
        let header = VfsHeader::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        // Check for error
        if header.msg_type == msg::RESULT {
            let code = self.parse_result_code(&resp_buf[..resp_len]);
            return Err(VfsError::from_i32(code));
        }

        // Parse FILE_DATA
        if header.msg_type != msg::FILE_DATA {
            return Err(VfsError::InvalidMessage);
        }

        let (file_data, content) = FileData::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        Ok(FileContent {
            data: content,
            eof: file_data.eof != 0,
        })
    }

    /// Get file/directory info
    pub fn stat(&mut self, path: &[u8]) -> Result<FileStat, VfsError> {
        let seq_id = self.next_seq_id();
        let req = Stat::new(seq_id);

        let mut req_buf = [0u8; 256];
        let len = req.write_to(&mut req_buf, path)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let mut resp_buf = [0u8; 64];
        let resp_len = self.channel.recv(&mut resp_buf)
            .map_err(|_| VfsError::IoError)?;

        // Parse header
        let header = VfsHeader::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        // Check for error
        if header.msg_type == msg::RESULT {
            let code = self.parse_result_code(&resp_buf[..resp_len]);
            return Err(VfsError::from_i32(code));
        }

        // Parse FILE_INFO
        if header.msg_type != msg::FILE_INFO {
            return Err(VfsError::InvalidMessage);
        }

        let info = FileInfo::from_bytes(&resp_buf[..resp_len])
            .ok_or(VfsError::InvalidMessage)?;

        Ok(FileStat {
            file_type: info.file_type,
            size: info.size,
        })
    }

    /// Create a directory
    pub fn mkdir(&mut self, path: &[u8]) -> Result<(), VfsError> {
        let seq_id = self.next_seq_id();
        let req = MakeDir::new(seq_id);

        let mut req_buf = [0u8; 256];
        let len = req.write_to(&mut req_buf, path)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let mut resp_buf = [0u8; 32];
        let resp_len = self.channel.recv(&mut resp_buf)
            .map_err(|_| VfsError::IoError)?;

        self.check_result(&resp_buf[..resp_len])
    }

    /// Remove a file
    pub fn remove_file(&mut self, path: &[u8]) -> Result<(), VfsError> {
        self.remove_internal(path, remove_flags::FILE)
    }

    /// Remove an empty directory
    pub fn remove_dir(&mut self, path: &[u8]) -> Result<(), VfsError> {
        self.remove_internal(path, remove_flags::DIR)
    }

    /// Remove file or directory
    fn remove_internal(&mut self, path: &[u8], flags: u8) -> Result<(), VfsError> {
        let seq_id = self.next_seq_id();
        let req = Remove::new(seq_id, flags);

        let mut req_buf = [0u8; 256];
        let len = req.write_to(&mut req_buf, path)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let mut resp_buf = [0u8; 32];
        let resp_len = self.channel.recv(&mut resp_buf)
            .map_err(|_| VfsError::IoError)?;

        self.check_result(&resp_buf[..resp_len])
    }

    /// Write data to a file (create if needed)
    pub fn write_file(&mut self, path: &[u8], data: &[u8], flags: u8) -> Result<(), VfsError> {
        let seq_id = self.next_seq_id();
        let req = WriteFile::new(seq_id, 0, flags);

        let mut req_buf = [0u8; 4096 + 256];
        let len = req.write_to(&mut req_buf, path, data)
            .ok_or(VfsError::InvalidPath)?;

        self.channel.send(&req_buf[..len])
            .map_err(|_| VfsError::IoError)?;

        crate::ipc::wait_one(self.channel.handle())
            .map_err(|_| VfsError::IoError)?;

        let mut resp_buf = [0u8; 32];
        let resp_len = self.channel.recv(&mut resp_buf)
            .map_err(|_| VfsError::IoError)?;

        self.check_result(&resp_buf[..resp_len])
    }

    /// Copy a file (read from src, write to dst)
    pub fn copy_file(&mut self, src: &[u8], dst: &[u8]) -> Result<(), VfsError> {
        // Read source file
        let mut content_buf = [0u8; 4096 + 64];
        let content = self.read_file(src, 0, 4096, &mut content_buf)?;

        // Write to destination
        self.write_file(dst, content.data, write_flags::CREATE | write_flags::TRUNCATE)
    }

    /// Check RESULT response for errors
    fn check_result(&self, buf: &[u8]) -> Result<(), VfsError> {
        let header = VfsHeader::from_bytes(buf)
            .ok_or(VfsError::InvalidMessage)?;

        if header.msg_type != msg::RESULT {
            return Err(VfsError::InvalidMessage);
        }

        let code = self.parse_result_code(buf);
        if code == error::OK {
            Ok(())
        } else {
            Err(VfsError::from_i32(code))
        }
    }

    /// Parse result code from RESULT message
    fn parse_result_code(&self, buf: &[u8]) -> i32 {
        if buf.len() >= VfsHeader::SIZE + 4 {
            i32::from_le_bytes([
                buf[VfsHeader::SIZE],
                buf[VfsHeader::SIZE + 1],
                buf[VfsHeader::SIZE + 2],
                buf[VfsHeader::SIZE + 3],
            ])
        } else {
            error::INTERNAL
        }
    }
}
