1) Memory model and address spaces
This is where “rock solid” kernels are made.
What to harden next
One canonical page-table API with a provable set of invariants:
map/unmap are the only mutators
refcounts (or ownership) for page frames are explicit
all user mappings are tracked (so unmap can’t leak)
User memory pinning rules for IPC and DMA:
either “always copy” (simplest/robust)
or “pin + map” with a strict lifetime token so you can’t forget to unpin
Direction
Make memory a first-class subsystem with an internal “VM object” story:
PageFrame (owned physical page)
VmObject (collection of frames)
VmMapping (vaddr → VM object slice)
AddressSpace (page table + mapping set)
If you keep this tight, everything else (IPC zero-copy, DMA, file cache later) becomes safer without growing the kernel.
2) IPC: define semantics like an RFC, then enforce them with types
Microkernels live or die by IPC correctness.
What to decide explicitly
Delivery semantics: reliable vs best-effort; ordering guarantees per endpoint
Backpressure: bounded queues, what happens on full
Cancellation: what happens to a blocked sender/receiver if the other side dies
Zero-copy: only via explicit “transfer objects” (shared memory handle), never “implicit pointer passing”
Direction
Treat IPC endpoints as kernel objects with:
a strict state machine (Connected/Closed/Draining)
bounded queues and explicit overflow behavior
structured message formats (even if it’s “just bytes” now)
And build “never wedge the kernel” invariants:
“IPC cannot allocate unbounded memory”
“IPC cannot hold locks across block”
“IPC cannot depend on user memory staying mapped”
3) Object model + capabilities: make authority flow only through handles
Your object-centric syscall surface is already the right architecture. Next step is to make it the only security boundary that matters.
Tighten it
Coarse Capabilities are fine as a “privilege class gate”
But the real enforcement should be:
a handle grants specific rights on a specific object
rights are not “global MMIO yes/no”
Direction
Move toward:
Handle<T, Rights> conceptually (even if erased at runtime)
“minted” objects for dangerous things:
MMIO region object
IRQ object
DMA pool/queue object
page frame / shared memory object
This keeps the kernel small because everything becomes the same pattern: object + rights + state machine.
4) Interrupts, timers, and timekeeping
This becomes portability glue and correctness glue.
What to harden
Interrupt dispatch must be non-blocking (log, enqueue event, return)
Timer semantics: monotonic time source, sleep accuracy, overflow behavior
Per-CPU interrupt accounting: you’ll want this for storm control and debugging
Direction for x86 + ARM64
Define a tiny arch trait surface:
Arch::irq_enable/disable
Arch::send_ipi(cpu, reason)
Arch::timer_now / timer_set_deadline
Arch::interrupt_controller abstraction:
GICv2/v3 on ARM
APIC/x2APIC on x86
Everything above that should be arch-neutral “events”.
5) Boot and hardware description: this is where “more than a Pi” lives
To support “more than a Raspberry Pi”, you need to not bake in one discovery model.
Direction: support both worlds cleanly
ARM64: Device Tree (DT) is the common reality for boards
x86_64: ACPI + PCI enumeration is the common reality (and UEFI boot)
So:
A single internal representation: your “device tree object model”
Two front-ends:
DT parser → internal tree
ACPI tables + PCI scan → internal tree
Then your devd/service model stays the same across platforms:
“device appears in tree → driver match → driver binds → exposes objects”
This is how you keep portability without turning the kernel into #ifdef soup.
6) Drivers: keep the kernel driverless, but give user drivers safe superpowers
If you want “compact + unbreakable”, drivers must be:
user mode
restartable
capability-scoped
able to DMA without shooting your foot off
The critical design choice
Don’t give drivers raw pointers to “kernel memory mapped to device”.
Give them:
an MMIO mapping object (bounded ranges)
a DMA buffer object (explicit lifecycle + sync hooks)
an IRQ endpoint object
And a strict rule:
“driver can’t wedge kernel by exhausting resources”
bounded allocations
timeouts
quotas per process
7) Portability strategy that won’t explode your codebase
If you want x86 + ARM64 and “more than one board”, make the layering explicit:
Recommended layering
arch/ (AArch64, x86_64): traps, context switch, page tables, irq/timer/IPI
platform/ (boards/machines): early boot, UART choice, “where is the DT blob”, etc.
hal/ (devices): common driver-facing abstractions (interrupt controller, timer, PCI)
kernel/ (pure): scheduler, ipc, objects, vmm, uaccess, supervision hooks
The rule: kernel/ never imports platform/.
8) Testing for correctness: make bugs impossible to hide
This is the fastest way to get “unbreakable” without writing a million lines.
Direction
Deterministic concurrency testing of the scheduler + wake/queue logic using a Loom-like approach (even if you build a tiny in-house harness).
Property tests for state machines:
generate random transition sequences
assert invariants always hold
Fuzz your syscall decoding and uaccess boundaries (especially copyin/out and object operations).
Run the kernel in QEMU early for both:
virt AArch64
q35 x86_64
Prefer virtio devices for early portability (virtio-net, virtio-blk) so you aren’t blocked on one SoC.
If you do only one thing: add debug-only invariant checks everywhere and make them cheap enough to leave on in dev builds.
A concrete “next direction” roadmap (compact + rock solid)
Finish SMP exclusivity + TLB shootdowns (correct, then fast)
Solidify VM + uaccess contracts (pin/copy semantics, no TOCTOU surprises)
Make objects the only authority (rights + state machine per object)
Build device-model unification (DT + ACPI/PCI → internal tree)
Run on QEMU virt targets with virtio (AArch64 + x86_64)
Move drivers to userland with strong capability objects (restartable, bounded)
That sequence keeps kernel size down while increasing confidence.
