# State Machines

Every boundary in the system has an explicit state machine. This document catalogs them.

## Kernel State Machines

### Bus Controller

Each hardware bus (PCIe, USB) has a kernel-side controller.

```
States:
  SAFE       - Bus in safe mode, no DMA possible
  CLAIMED    - Owned by devd, normal operation
  RESETTING  - Nuclear reset in progress

                         ┌───────────────────┐
                         │                   │
              boot       │       SAFE        │◄──────────────┐
             ──────────► │                   │               │
                         │  bus_master=off   │               │
                         └─────────┬─────────┘               │
                                   │                         │
                      port_connect │                         │ reset_complete
                                   ▼                         │
                         ┌───────────────────┐               │
                         │                   │               │
                         │     CLAIMED       │               │
                         │                   │               │
                         │  owner=Some(port) │               │
                         └─────────┬─────────┘               │
                                   │                         │
              owner_disconnect OR  │                         │
              Reset message        │                         │
                                   ▼                         │
                         ┌───────────────────┐               │
                         │                   │               │
                         │    RESETTING      ├───────────────┘
                         │                   │
                         │  [reset sequence] │
                         └───────────────────┘
```

| Current | Event | Next | Action |
|---------|-------|------|--------|
| SAFE | port_connect(port) | CLAIMED | owner = port |
| CLAIMED | owner_disconnect | RESETTING | begin reset sequence |
| CLAIMED | BusControlMsg::Reset | RESETTING | begin reset sequence |
| RESETTING | reset_complete | SAFE | clear resources |

### Process

```
         spawn()     ┌─────────┐
        ──────────►  │ CREATED │
                     └────┬────┘
                          │ loaded
                          ▼
        ┌───────────►┌─────────┐◄───────────┐
        │            │ RUNNABLE│            │
        │            └────┬────┘            │
        │                 │ scheduled       │ unblocked
yielded │                 ▼                 │
        │            ┌─────────┐            │
        │            │ RUNNING │            │
        │            └──┬───┬──┘            │
        │               │   │               │
        └───────────────┘   │ wait          │
                            ▼               │
                       ┌─────────┐          │
                       │ BLOCKED ├──────────┘
                       └────┬────┘
                            │ exit
                            ▼
                       ┌─────────┐
                       │ ZOMBIE  │ ──► reaped
                       └─────────┘
```

---

## devd State Machines

### Device Lifecycle

```
           enumerate()   ┌────────────┐
            ──────────►  │ DISCOVERED │◄─────────────────┐
                         └─────┬──────┘                  │
                               │ driver_match           │ clean_unbind
                               ▼                         │
                         ┌───────────┐                   │
                         │  BINDING  │                   │
                         └──┬─────┬──┘                   │
                            │     │ bind_fail            │
                  bind_ok   │     │ or timeout           │
                            ▼     │                      │
        ┌───────────►┌───────────┐│                      │
        │            │   BOUND   ├┼──────────────────────┘
        │            └─────┬─────┘│
        │                  │      │
        │  fault recovered │ driver_crash / hw_fault
        │                  ▼
        │            ┌───────────┐
        │            │  FAULTED  │
        │            └─────┬─────┘
        │                  │ retry_timer
        │                  ▼
        │            ┌───────────┐
        └────────────┤ RESETTING │
         reset_ok    └─────┬─────┘
                           │ max_escalation
                           ▼
                     ┌───────────┐
                     │   DEAD    │
                     └───────────┘
```

| Current | Event | Next | Action |
|---------|-------|------|--------|
| DISCOVERED | driver_match | BINDING | spawn driver |
| BINDING | bind_complete | BOUND | enable bus mastering |
| BINDING | bind_failed/timeout | FAULTED | start fault tracker |
| BOUND | driver_exit(clean) | DISCOVERED | disable bus mastering |
| BOUND | driver_crash | FAULTED | escalate |
| FAULTED | retry_timer | RESETTING | reset at escalation level |
| FAULTED | max_escalation | DEAD | mark dead |
| RESETTING | reset_success | BINDING | re-bind |
| DEAD | manual_reset | DISCOVERED | re-enumerate |

### Fault Escalation

```
       first_fault  ┌───────────────┐ ◄─── quiet_period (60s) resets here
         ─────────► │ DEVICE_RESET  │
                    └───────┬───────┘
                            │ 3 failures
                            ▼
                    ┌───────────────┐
                    │ LINK_RETRAIN  │
                    └───────┬───────┘
                            │ 2 failures
                            ▼
                    ┌───────────────┐
                    │ SEGMENT_RESET │
                    └───────┬───────┘
                            │ 2 failures
                            ▼
                    ┌───────────────┐
                    │  BUS_RESET    │ ← kernel nuclear reset
                    └───────┬───────┘
                            │ 1 failure
                            ▼
                    ┌───────────────┐
                    │     DEAD      │
                    └───────────────┘
```

Backoff doubles each attempt: 100ms → 200ms → 400ms → ... → 5000ms max.
After 60s of quiet operation, escalation resets to DEVICE_RESET.

---

## Driver State Machines

### Driver Connection (to devd)

```
                spawn    ┌──────────┐
               ───────►  │ STARTING │
                         └────┬─────┘
                              │ connect to devd
                              ▼
                         ┌────────────┐
                         │REGISTERING │
                         └────┬───────┘
                              │ registered
                              ▼
        ┌───────────────►┌──────────┐
        │                │  IDLE    │
        │                └────┬─────┘
        │                     │ bind_request
        │                     ▼
        │                ┌──────────┐
        │                │ BINDING  │
        │                └──┬────┬──┘
        │       bind_ok     │    │ reject
        │                   ▼    │
        │      ┌─────────────────────────┐
        │      │       OPERATING         │◄──┐
        │      └─────┬──────────┬────────┘   │ recovered
        │            │          │            │
 unbind │            │    fault │            │
        │            │          ▼            │
        └────────────┘    ┌───────────┐      │
                          │ RECOVERING├──────┘
                          └─────┬─────┘
                                │ unrecoverable
                                ▼
                          ┌───────────┐
                          │  STOPPED  │
                          └───────────┘
```

### Hardware Access (per device)

```
          hw_acquired    ┌─────────────┐
            ──────────►  │INITIALIZING │
                         └──────┬──────┘
                                │ init_complete
                                ▼
        ┌───────────────►┌───────────┐
        │                │   READY   │
        │                └─────┬─────┘
        │                      │ cmd_start
        │      cmd_complete    │
        │                      ▼
        │                ┌───────────┐
        └────────────────┤   BUSY    │
                         └─────┬─────┘
                               │ hw_error
                               ▼
                         ┌───────────┐
          ┌──────────────┤   ERROR   │
          │  reset_ok    └─────┬─────┘
          │                    │ reset_failed
          ▼                    ▼
     ┌───────────┐       ┌───────────┐
     │   READY   │       │   DEAD    │ ──► report to devd
     └───────────┘       └───────────┘
```

---

## Rules for New Components

When adding any component that communicates externally:

1. **Define the state machine first** - Draw states, transitions, invariants
2. **Implement with explicit state** - `enum MyState { ... }`
3. **Make state observable** - `.state()` getter, publish to /hw tree
4. **Make state serializable** - For live update support

```rust
enum MyState { Initial, Working, Failed, Done }

struct MyComponent {
    state: MyState,
}

impl MyComponent {
    fn handle_event(&mut self, event: Event) -> Result<()> {
        self.state = match (self.state, event) {
            (Initial, Start) => Working,
            (Working, Complete) => Done,
            (Working, Error) => Failed,
            _ => return Err(InvalidTransition),
        };
        Ok(())
    }
}
```

---

## Implementation Files

| State Machine | File |
|---------------|------|
| Bus Controller | `src/kernel/bus.rs` |
| Process | `src/kernel/task.rs` |
| Device (devd) | `user/driver/devd/src/main.rs` |
| FaultTracker | `user/driver/devd/src/main.rs` |
