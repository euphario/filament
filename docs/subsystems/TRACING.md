# Kernel Tracing Framework (ktrace)

## Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Span identification | Implicit call depth | Simple, no explicit parent tracking needed |
| Timing | Entry + exit timestamps | Calculate duration offline, flexible |
| Buffer | Separate from klog | High-frequency traces won't push out logs |
| Buffer size | 64KB (kernel), 32KB (userspace) | Same as klog |
| Format | Binary | Compact, high frequency, format on drain |

## Purpose

Tracing captures **execution flow** - enter/exit of functions, call depth, timing.
Different from logging which captures **events**.

```
Logging:  "firmware loaded successfully"
Tracing:  load_firmware() took 234ms, called load_patch() which took 15ms, ...
```

## Output Format (Text, after formatting)

```
00012483 >  fw load_firmware dev=0001:01:00.0
00012485 >    fw load_patch
00012486 >      mcu send_cmd cmd=0x10 len=68
00012487 <      mcu send_cmd 1.2ms ok
00012500 <    fw load_patch 15ms ok
00012501 >    fw load_ram component=WM
00012502 >      dma alloc_ring size=65536
00012503 <      dma alloc_ring 0.8ms ok
00012734 <    fw load_ram 233ms ok
00012735 <  fw load_firmware 252ms ok
```

Key elements:
- `>` = enter, `<` = exit
- Indentation shows call depth (2 spaces per level)
- Duration shown on exit
- Result (ok/err) shown on exit

## Binary Record Format

### Trace Event (16 bytes fixed header + variable data)

```rust
#[repr(C)]
struct TraceEvent {
    ts_us: u32,       // Microseconds since boot (4 bytes)
    flags: u8,        // Event type + depth (see below)
    subsys_len: u8,   // Length of subsystem string
    name_len: u8,     // Length of span name
    kv_count: u8,     // Number of key-value pairs
    // Followed by:
    // - subsys string (subsys_len bytes)
    // - name string (name_len bytes)
    // - key-value pairs (same format as klog)
}

// flags byte:
// bits 0-5: depth (0-63)
// bit 6: 0=enter, 1=exit
// bit 7: 0=ok, 1=error (only for exit)
```

### Why microseconds?

Tracing needs finer granularity than logging:
- Logging: millisecond precision sufficient
- Tracing: microsecond precision for measuring fast operations

32-bit microseconds = ~71 minutes before wrap, sufficient for most traces.

## API

### Kernel (`src/ktrace.rs`)

```rust
// Manual enter/exit
trace_enter!("fw", "load_firmware"; dev = "0001:01:00.0");
// ... do work ...
trace_exit!("fw", "load_firmware");  // ok
trace_exit_err!("fw", "load_firmware"; err = "timeout");  // error

// RAII guard (preferred)
fn load_firmware(dev: &Device) -> Result<()> {
    let _span = span!("fw", "load_firmware"; dev = dev.bdf());

    load_patch()?;  // nested span inside
    load_ram()?;

    Ok(())
    // _span dropped here, auto-emits exit with ok/err based on result
}

// With explicit result capture
fn load_firmware(dev: &Device) -> Result<()> {
    let span = span!("fw", "load_firmware"; dev = dev.bdf());

    let result = do_load();

    span.finish_with(result)  // Emits exit with ok/err
}
```

### Userspace (`userlib/src/utrace.rs`)

Same API, different backend (syscall drain).

```rust
use userlib::{span, trace_enter, trace_exit};

fn init_dma(dev: &Mt7996) -> Result<()> {
    let _span = span!("dma", "init"; dev = dev.bdf());
    // ...
}
```

## Span Guard Implementation

```rust
pub struct Span {
    subsys: &'static str,
    name: &'static str,
    depth: u8,
    finished: bool,
}

impl Span {
    pub fn enter(subsys: &'static str, name: &'static str, kvs: &[(&str, Value)]) -> Self {
        let depth = increment_depth();
        emit_enter(subsys, name, depth, kvs);
        Self { subsys, name, depth, finished: false }
    }

    pub fn exit_ok(mut self) {
        emit_exit(self.subsys, self.name, self.depth, false);
        self.finished = true;
    }

    pub fn exit_err(mut self, kvs: &[(&str, Value)]) {
        emit_exit_with_kvs(self.subsys, self.name, self.depth, true, kvs);
        self.finished = true;
    }
}

impl Drop for Span {
    fn drop(&mut self) {
        if !self.finished {
            // Auto-exit with ok (assume success if not explicitly failed)
            emit_exit(self.subsys, self.name, self.depth, false);
        }
        decrement_depth();
    }
}
```

## Depth Tracking

Global atomic counter (single-core for now, per-CPU later):

```rust
static TRACE_DEPTH: AtomicU8 = AtomicU8::new(0);

fn increment_depth() -> u8 {
    TRACE_DEPTH.fetch_add(1, Ordering::Relaxed)
}

fn decrement_depth() {
    TRACE_DEPTH.fetch_sub(1, Ordering::Relaxed);
}
```

## Ring Buffer

Same design as klog but separate instance:

```rust
static mut TRACE_RING: TraceRing = TraceRing::new();  // 64KB
```

## Drain/Formatting

```rust
// Drain and format one event
pub fn drain_one() -> bool {
    // Read binary event
    // Format to text with indentation based on depth
    // Output to UART
}

// Dump all pending traces
pub fn flush() {
    while drain_one() {}
}
```

## Integration with wifid3

Example of how wifid3 would use this:

```rust
fn load_firmware(&mut self) -> Result<()> {
    let _span = span!("fw", "load_firmware");

    self.load_patch()?;
    self.load_ram("WM")?;
    self.load_ram("WA")?;

    Ok(())
}

fn load_patch(&mut self) -> Result<()> {
    let _span = span!("fw", "load_patch");

    {
        let _span = span!("mcu", "patch_sem_ctrl"; cmd = utrace::hex32(0x10));
        self.mcu_patch_sem_ctrl()?;
    }

    // ... send patch chunks ...

    {
        let _span = span!("mcu", "patch_start"; cmd = utrace::hex32(0x05));
        self.mcu_patch_start()?;
    }

    Ok(())
}

fn mcu_send_cmd(&mut self, cmd: u32, data: &[u8]) -> Result<()> {
    let _span = span!("mcu", "send_cmd"; cmd = utrace::hex32(cmd), len = data.len());

    // ... actual send ...

    // If this returns Err, span will auto-exit with error
    self.wait_for_response()
}
```

Output when firmware loading fails:

```
00000000 >  fw load_firmware
00000001 >    fw load_patch
00000002 >      mcu patch_sem_ctrl cmd=0x00000010
00000003 <      mcu patch_sem_ctrl 1.2ms ok
00000015 >      mcu send_chunk idx=0 len=4096
00000016 <      mcu send_chunk 0.8ms ok
...
00000150 >      mcu patch_start cmd=0x00000005
00000151 <      mcu patch_start 0.9ms ok
00000152 <    fw load_patch 151ms ok
00000153 >    fw load_ram component=WM
00000154 >      mcu init_download addr=0x00900000 len=2655416
00000155 <      mcu init_download 1.1ms ok
00000156 >      mcu send_chunk idx=0 len=4096
00000157 >        dma kick_queue q=FWDL cpu_idx=1
00000158 <        dma kick_queue 0.1ms ok
00000200 <      mcu send_chunk 43ms err timeout
00000201 <    fw load_ram 47ms err
00000202 <  fw load_firmware 198ms err
```

Now you can see exactly where it failed and how long each step took.

## Control via devd

Same model as klog:
- `/obj/ktrace` - read trace buffer
- `/obj/tracectl` - enable/disable, set filters

```
# Enable tracing for fw and mcu subsystems
trace on fw,mcu

# Trace with depth limit
trace on fw --max-depth=3

# Dump current trace buffer
trace dump

# Clear buffer
trace clear
```

## Files

| File | Purpose |
|------|---------|
| `src/ktrace.rs` | Kernel tracing |
| `user/userlib/src/utrace.rs` | Userspace tracing |
| `docs/TRACING.md` | This document |
