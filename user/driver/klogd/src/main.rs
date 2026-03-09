#![no_std]
#![no_main]

extern crate libsys;

use libos::bus::{BusCtx, BusError, ConfigKey, Disposition, Driver, BusMsg};
use libos::bus_runtime::driver_main;
use libsys::syscall::{self, Handle, ObjectType};

const TAG_KLOG: u32 = 0x4B4C; // "KL"

struct KlogDriver {
    klog_handle: Option<Handle>,
}

impl KlogDriver {
    fn new() -> Self {
        Self { klog_handle: None }
    }

    fn drain_to_nowhere(&mut self) {
        let Some(handle) = self.klog_handle else { return };
        let mut buf = [0u8; 1024];
        // Read and discard up to 32 records per event to keep drain_cursor moving
        for _ in 0..32 {
            match syscall::read(handle, &mut buf) {
                Ok(0) | Err(_) => break,
                Ok(_) => {}
            }
        }
    }
}

const KLOG_CONFIG_KEYS: &[ConfigKey] = &[
    ConfigKey::read_only(b"status"),
];

impl Driver for KlogDriver {
    fn reset(&mut self, ctx: &mut dyn BusCtx) -> Result<(), BusError> {
        // Claim the kernel klog bus — this suppresses UART drain
        let (_bus_id, _info) = ctx.claim_kernel_bus(b"/klog:0")?;

        // Open a Klog handle for reading records
        match syscall::open(ObjectType::Klog, &[]) {
            Ok(handle) => {
                ctx.watch_handle(handle, TAG_KLOG)?;
                self.klog_handle = Some(handle);
            }
            Err(_) => {
                // Klog handle not critical — bus claim alone suppresses UART drain
            }
        }

        Ok(())
    }

    fn command(&mut self, _msg: &BusMsg, _ctx: &mut dyn BusCtx) -> Disposition {
        Disposition::Forward
    }

    fn handle_event(&mut self, tag: u32, _handle: Handle, _ctx: &mut dyn BusCtx) {
        if tag == TAG_KLOG {
            // Drain records so the read_pos advances (prevents KlogObject from
            // always reporting readable and spinning the Mux).
            self.drain_to_nowhere();
        }
    }

    fn config_keys(&self) -> &[ConfigKey] {
        KLOG_CONFIG_KEYS
    }

    fn config_get(&self, key: &[u8], buf: &mut [u8]) -> usize {
        match key {
            b"status" => {
                let msg = b"active";
                let len = msg.len().min(buf.len());
                buf[..len].copy_from_slice(&msg[..len]);
                len
            }
            _ => 0,
        }
    }
}

#[unsafe(no_mangle)]
fn main() {
    driver_main(b"klogd", KlogDriver::new());
}
