//! CPU Power Management
//!
//! Governor-based CPU power management. Runs from the timer IRQ on CPU 0.
//! Samples per-CPU utilization every 100ms and adjusts the number of active
//! CPUs in the scheduler based on the current governor policy.
//!
//! The time source for utilization sampling is the ARM generic timer counter
//! (CNTPCT_EL0), which runs at a fixed frequency independent of CPU frequency.

use crate::kernel::lock::{SpinLock, lock_class};
use crate::kernel::percpu;
use core::sync::atomic::{AtomicU8, Ordering};

/// Governor modes
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Governor {
    Performance = 0,
    Balanced = 1,
    Powersave = 2,
}

impl Governor {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Governor::Performance),
            1 => Some(Governor::Balanced),
            2 => Some(Governor::Powersave),
            _ => None,
        }
    }
}

/// Tick interval for sampling (100ms = every 10 timer ticks at 10ms/tick)
const SAMPLE_INTERVAL: u8 = 10;

/// Governor set atomically from syscall context (avoids lock ordering with BUS lock).
/// tick() reads this and applies the change under POWER lock.
static GOVERNOR: AtomicU8 = AtomicU8::new(Governor::Performance as u8);

/// Target OPP set atomically from bus controller context (same pattern as GOVERNOR).
/// tick() reads this and applies the change under POWER lock.
static TARGET_OPP: AtomicU8 = AtomicU8::new(0xFF); // 0xFF = governor-controlled

struct PowerManager {
    governor: Governor,
    num_active: u8,
    current_opp: u8,
    sample_counter: u8,
    prev_ticks: [u64; percpu::MAX_CPUS],
    prev_idle: [u64; percpu::MAX_CPUS],
    util: [u8; percpu::MAX_CPUS],
}

impl PowerManager {
    const fn new() -> Self {
        Self {
            governor: Governor::Performance,
            num_active: 1,
            current_opp: 0xFF, // uninitialized until first tick
            sample_counter: 0,
            prev_ticks: [0; percpu::MAX_CPUS],
            prev_idle: [0; percpu::MAX_CPUS],
            util: [0; percpu::MAX_CPUS],
        }
    }
}

static POWER: SpinLock<PowerManager> = SpinLock::new(lock_class::UNORDERED, PowerManager::new());

/// Called from timer IRQ on CPU 0 every tick.
/// Samples utilization every SAMPLE_INTERVAL ticks and applies governor policy
/// for both CPU parking and DVFS frequency selection.
pub fn tick() {
    use crate::platform::current::cpufreq;

    let mut pm = POWER.lock();
    pm.sample_counter += 1;
    if pm.sample_counter < SAMPLE_INTERVAL {
        return;
    }
    pm.sample_counter = 0;

    // Pick up governor changes from set_governor() (lock-free handoff)
    let new_gov = GOVERNOR.load(Ordering::Relaxed);
    if let Some(gov) = Governor::from_u8(new_gov) {
        pm.governor = gov;
    }

    // Sync current OPP from platform on first tick
    if pm.current_opp == 0xFF {
        pm.current_opp = cpufreq::current_opp() as u8;
    }

    let num_cpus = percpu::num_online_cpus() as usize;
    let max_opp = (cpufreq::opp_count() as u8).saturating_sub(1);

    // Sample per-CPU utilization from tick counters
    for cpu in 0..num_cpus {
        if let Some(data) = percpu::try_get_cpu_data(cpu) {
            let ticks = data.tick_count.load(core::sync::atomic::Ordering::Relaxed);
            let idle = data.idle_ticks.load(core::sync::atomic::Ordering::Relaxed);

            let delta_ticks = ticks.wrapping_sub(pm.prev_ticks[cpu]);
            let delta_idle = idle.wrapping_sub(pm.prev_idle[cpu]);

            pm.prev_ticks[cpu] = ticks;
            pm.prev_idle[cpu] = idle;

            if delta_ticks > 0 {
                let busy = delta_ticks.saturating_sub(delta_idle);
                pm.util[cpu] = ((busy * 100) / delta_ticks).min(100) as u8;
            } else {
                pm.util[cpu] = 0;
            }
        }
    }

    // Apply governor policy — CPU parking
    let target = match pm.governor {
        Governor::Performance => num_cpus as u8,
        Governor::Balanced => {
            let any_high = pm.util[..num_cpus].iter().any(|&u| u > 80);
            let all_low = pm.util[..num_cpus].iter().all(|&u| u < 30);
            if any_high {
                (pm.num_active + 1).min(num_cpus as u8)
            } else if all_low && pm.num_active > 1 {
                pm.num_active - 1
            } else {
                pm.num_active
            }
        }
        Governor::Powersave => {
            let active = pm.util[..num_cpus].iter().filter(|&&u| u > 10).count() as u8;
            active.max(1)
        }
    };

    // Use try_scheduler: we're in timer IRQ context, scheduler lock may be
    // held by the interrupted syscall. If locked, defer to next sample.
    if let Some(mut sched) = crate::kernel::task::try_scheduler() {
        if target != pm.num_active || sched.policy.num_active() != pm.num_active {
            pm.num_active = target;
            sched.policy.set_num_active(target);
        }
    } else if target != pm.num_active {
        // Scheduler locked — record intent, apply on next successful lock
        pm.num_active = target;
    }

    // Apply governor policy — DVFS frequency selection
    // Check for manual OPP override from SetOpp command
    let manual_opp = TARGET_OPP.load(Ordering::Relaxed);
    let target_opp = if manual_opp != 0xFF {
        // Manual override active — use it and clear
        TARGET_OPP.store(0xFF, Ordering::Relaxed);
        manual_opp.min(max_opp)
    } else {
        // Governor-controlled frequency
        match pm.governor {
            Governor::Performance => max_opp,
            Governor::Balanced => {
                let any_high = pm.util[..num_cpus].iter().any(|&u| u > 80);
                let all_low = pm.util[..num_cpus].iter().all(|&u| u < 30);
                if any_high {
                    (pm.current_opp + 1).min(max_opp)
                } else if all_low {
                    pm.current_opp.saturating_sub(1)
                } else {
                    pm.current_opp
                }
            }
            Governor::Powersave => {
                // Find minimum OPP that keeps max utilization under 80%.
                // Start from lowest and step up until headroom is sufficient.
                let max_util = pm.util[..num_cpus].iter().copied().max().unwrap_or(0);
                if max_util > 80 {
                    (pm.current_opp + 1).min(max_opp)
                } else if max_util < 30 {
                    pm.current_opp.saturating_sub(1)
                } else {
                    pm.current_opp
                }
            }
        }
    };

    if target_opp != pm.current_opp {
        cpufreq::set_opp(target_opp as usize);
        pm.current_opp = target_opp;
    }

    // Push CpuStateUpdate to CPU bus owner if claimed
    let update = build_state_update_inner(&pm, num_cpus);
    drop(pm);
    push_cpu_state_to_bus(&update);
}

/// Set the governor mode. Returns true if valid.
///
/// Called from BusController::handle_message() with BUS lock held,
/// so we use an atomic store instead of acquiring the POWER lock.
/// tick() picks up the change on its next sample interval.
pub fn set_governor(g: u8) -> bool {
    if Governor::from_u8(g).is_some() {
        GOVERNOR.store(g, Ordering::Relaxed);
        true
    } else {
        false
    }
}

/// Set target OPP (frequency). Returns true if valid index.
///
/// Called from BusController::handle_message() with BUS lock held.
/// tick() picks up the change on its next sample interval.
pub fn set_target_opp(index: u8) -> bool {
    let count = crate::platform::current::cpufreq::opp_count();
    if (index as usize) < count {
        TARGET_OPP.store(index, Ordering::Relaxed);
        true
    } else {
        false
    }
}

/// Build the 12-byte CpuStateUpdate payload.
/// [governor(1), num_cpus(1), active(1), current_opp(1), freq_khz_le32(4), util0..3(4)]
fn build_state_update_inner(pm: &PowerManager, num_cpus: usize) -> [u8; 12] {
    use crate::platform::current::cpufreq;
    let mut buf = [0u8; 12];
    buf[0] = pm.governor as u8;
    buf[1] = num_cpus as u8;
    buf[2] = pm.num_active;
    buf[3] = pm.current_opp;
    let freq = cpufreq::current_freq_khz();
    buf[4..8].copy_from_slice(&freq.to_le_bytes());
    for i in 0..num_cpus.min(4) {
        buf[8 + i] = pm.util[i];
    }
    buf
}

/// Push CpuStateUpdate to the CPU bus owner channel (if claimed).
fn push_cpu_state_to_bus(update: &[u8; 12]) {
    use crate::kernel::bus::{self, BusState};
    use crate::kernel::ipc::{self, Message, MessageType, ChannelId, waker, WakeReason};

    // Phase 1: Find owner channel under bus lock
    let owner_ch: Option<ChannelId> = bus::with_bus_registry(|registry| {
        for bus in registry.iter_mut() {
            if bus.bus_type == bus::BusType::Cpu {
                if bus.state() != BusState::Claimed {
                    return None;
                }
                return bus.owner_ch;
            }
        }
        None
    });

    // Phase 2: Send + wake outside bus lock (avoids BUS→OBJ_SERVICE lock ordering issue)
    let Some(ch) = owner_ch else { return };

    let mut msg = Message::new();
    msg.header.msg_type = MessageType::Data;
    msg.payload[0] = 32; // CpuStateUpdate
    msg.payload[1..13].copy_from_slice(update);
    msg.header.payload_len = 13;

    if let Ok(peer) = ipc::send_unchecked(ch, msg) {
        let wake_list = crate::kernel::object_service::object_service()
            .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
        waker::wake(&wake_list, WakeReason::Readable);
    }
}

/// Push OPP table to the CPU bus owner channel.
/// Called once when the CPU bus is claimed (from bus controller).
pub fn push_opp_table_to_bus(channel: crate::kernel::ipc::ChannelId) {
    use crate::kernel::ipc::{self, Message, MessageType, waker, WakeReason};
    use crate::platform::current::cpufreq;

    let count = cpufreq::opp_count();
    // Message: [msg_type(1), count(1), (freq_khz_le32(4)) * count]
    let payload_len = 2 + count * 4;

    let mut msg = Message::new();
    msg.header.msg_type = MessageType::Data;
    msg.payload[0] = 35; // OppTable
    msg.payload[1] = count as u8;
    for i in 0..count {
        let freq = cpufreq::opp_freq_khz(i);
        let off = 2 + i * 4;
        msg.payload[off..off + 4].copy_from_slice(&freq.to_le_bytes());
    }
    msg.header.payload_len = payload_len as u32;

    if let Ok(peer) = ipc::send_unchecked(channel, msg) {
        let wake_list = crate::kernel::object_service::object_service()
            .wake_channel(peer.task_id, peer.channel_id, abi::mux_filter::READABLE);
        waker::wake(&wake_list, WakeReason::Readable);
    }
}
