# libf — Filament Standard Library

## What libf Is

libf is the Filament equivalent of libc. It sits between raw kernel syscalls (userlib) and application code, providing the programming interface every program needs.

```
Application code (shell, drivers, daemons)
        ↓ uses
libf                ← programming interface (fmt, str, parse, fs, net...)
        ↓ uses
userlib             ← kernel interface (raw syscalls, IPC primitives, bus framework)
        ↓ calls
Kernel              ← 5 unified syscalls (open/read/write/map/close)
```

## Design Principle: Mirror Rust std Signatures

**libf intentionally mimics Rust's `std` API signatures.**

Rust's standard library is layered: `core` (no OS) → `alloc` (heap) → `std` (OS integration). Filament already has `core` and `alloc`. The missing piece is `std`, which provides OS abstractions like `File`, `TcpStream`, `Mutex`, and utility traits like `Read`/`Write`.

libf fills this gap with a deliberate strategy:

1. **Match std's method signatures** — When libf provides something std also provides, use the same function names, argument types, and return types. A `libf::fs::File` should have `.read()`, `.write()`, `.metadata()` with signatures matching `std::fs::File`.

2. **Match std's trait signatures** — `libf::io::Read` and `libf::io::Write` should have the same method signatures as their std counterparts. This means user code reads identically whether targeting libf or std.

3. **Match std's module layout** — `libf::io`, `libf::fs`, `libf::net`, `libf::fmt` mirror `std::io`, `std::fs`, `std::net`, `std::fmt`.

### Why

This creates a clean migration path. When Filament is mature enough to justify a full `std` port (registering `aarch64-unknown-filament` as a Rust target and implementing `std::sys::pal::filament`), the transition is mechanical:

- **Our code**: Change `use libf::` to `use std::` — the signatures already match.
- **Ecosystem unlock**: With real `std`, any crate on crates.io that targets `std` Just Works. This is the actual prize — `serde`, `regex`, `tokio`, and thousands of other crates become available.

The key insight: libf's implementations *become* the `std::sys::filament` platform layer. The work isn't throwaway — it's a rehearsal for the real thing.

### What This Means in Practice

**Do:**
- Name functions/methods the same as std (`trim`, `parse`, `read`, `write`)
- Use the same trait patterns (`impl Read for File`, `impl Write for File`)
- Return `Result<T, Error>` with error types that map to std conventions
- Organize modules to mirror std's layout

**Don't:**
- Don't import or depend on std (we're `no_std`)
- Don't add std compatibility shims or cfg gates yet — that's premature
- Don't force std's design where it doesn't fit Filament's model (Filament's object handles are cleaner than Unix file descriptors — lean into that)

## Current Contents (v0.1)

### `libf::fmt` — Formatting

- `StackStr` — 64-byte stack buffer implementing `core::fmt::Write` + `Display`
  - `StackStr::from_u32(val)`, `from_u64(val)`, `from_hex32(val)`, `from_hex64(val)`
- `format_u32_into(buf, val) -> usize` — write decimal into caller's buffer
- `format_u64_into(buf, val) -> usize`
- `format_hex32_into(buf, val) -> usize` — with `0x` prefix, minimal digits
- `format_hex64_into(buf, val) -> usize`

### `libf::str` — Byte-slice String Operations

- `trim(input) -> &[u8]` — trim ASCII whitespace both ends
- `trim_start(input) -> &[u8]`, `trim_end(input) -> &[u8]`
- `eq_ignore_ascii_case(a, b) -> bool`
- `starts_with_ignore_case(haystack, prefix) -> bool`
- `contains_ignore_ascii_case(haystack, needle) -> bool`
- `split_once(input, sep) -> (&[u8], &[u8])`
- `split_whitespace::<const MAX>(input) -> ([&[u8]; MAX], usize)`

### `libf::parse` — Number Parsing

- `parse_u32(input) -> Option<u32>` — decimal with overflow checking
- `parse_u64(input) -> Option<u64>`
- `parse_hex_u32(input) -> Option<u32>` — optional `0x` prefix
- `parse_hex_u64(input) -> Option<u64>`

### `libf::prelude` — Convenience Re-exports

Re-exports alloc types (`String`, `Vec`, `Box`, `format!`) and common libf/userlib symbols.

## Future Modules (Planned)

These will grow as the OS matures:

| Module | std Equivalent | Filament Backing |
|--------|---------------|-----------------|
| `libf::fs` | `std::fs` | `open(ObjectType::File)` + read/write/close |
| `libf::io` | `std::io` | `Read`/`Write` traits over object handles |
| `libf::net` | `std::net` | `open(ObjectType::TcpStream)` + read/write |
| `libf::process` | `std::process` | `exec()` + `wait()` |
| `libf::time` | `std::time` | `cntvct_el0` hardware counter |
| `libf::sync` | `std::sync` | Kernel-backed mutexes/condvars |

Filament's unified 5-syscall design maps naturally to std's abstractions:

```
std::fs::File::open()   →  syscall::open(ObjectType::File, ...)
std::fs::File::read()   →  syscall::read(handle, ...)
std::fs::File::write()  →  syscall::write(handle, ...)
drop(File)              →  syscall::close(handle)
```

## Relationship to std Port

```
Today:     core + alloc + libf (no_std, our code only)
                    ↓ when ecosystem compat is needed
Tomorrow:  core + alloc + std  (real target, crates.io unlocked)
                    ↑
           libf's implementations move into std::sys::pal::filament
```

The decision to port std is driven by a concrete need (wanting to use a specific crate), not by completeness for its own sake. Until then, libf grows module by module as the OS needs it.
