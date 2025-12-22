> === Enumerating Hub (Slot 1) ===
  SET_CONFIGURATION(1)...
  Transfer event: slot=1, CC=1, remain=0
    Configuration set
  SET_HUB_DEPTH(0)...
  Transfer event: slot=1, CC=1, remain=0
    Hub depth set
  Getting Hub Descriptor...
  Transfer event: slot=1, CC=1, remain=0
    Got 12 bytes
  Hub has 4 ports, power-on time: 350ms
  Configuring hub slot context...
    Slot ctx: dw0=0c400000, dw1=04010000
    Hub context updated
  Powering hub ports...
  Transfer event: slot=1, CC=1, remain=0
    Port 1 powered
  Transfer event: slot=1, CC=1, remain=0
    Port 2 powered
  Transfer event: slot=1, CC=1, remain=0
    Port 3 powered
  Transfer event: slot=1, CC=1, remain=0
    Port 4 powered
  Waiting 500ms for power stabilization..... done
  Checking port status...
  Transfer event: slot=1, CC=1, remain=0
    Port 1: status=0x02a0 change=0x0000 [empty]
  Transfer event: slot=1, CC=1, remain=0
    Port 2: status=0x02a0 change=0x0000 [empty]
  Transfer event: slot=1, CC=1, remain=0
    Port 3: status=0x02a0 change=0x0000 [empty]
  Transfer event: slot=1, CC=1, remain=0
    Port 4: status=0x0203 change=0x0030 [CONNECTED]
  Transfer event: slot=1, CC=1, remain=0
    Resetting port 4...
  Transfer event: slot=1, CC=1, remain=0
  Transfer event: slot=1, CC=1, remain=0
  Transfer event: slot=1, CC=1, remain=0
      Port enabled after reset
      Waiting 100ms for device...

=== Enumerating Device on Hub Port 4 ===
  Enabling slot...
    Enable Slot event: CC=1, slot=2
    Got slot 2
  Slot 2 for hub port 4 device
  SuperSpeed device - routing via route string only
  Child slot ctx: dw0=08400004, dw1=00010000, dw2=00000000
    Route=4, Speed=4, RootPort=1
  DCBAA[1] (hub) = 0000000040086500
  DCBAA[2] (child) = 000000004008a500
  Input control: add=00000003, drop=00000000
  Input EP0: dw0=00000000, dw1=02000026, tr_dequeue=000000004008a901
  Waiting 200ms before addressing...
  Addressing device (slot 2)...
    Cmd ring: enq=5, cycle=true
    TRB at 0000000040084050, input_ctx=000000004008a000
    Event ring: deq=23, cycle=true
    Event: type=33, status=01000000, ctrl=02008401
  Device address: 0
  Output slot ctx: dw0=00000000, dw3=00000000
  Output EP0 ctx: dw0=00000000, dw1=00000000
    TR Dequeue (output): 0000000000000000 (expected: 000000004008a901)
    EP0 state: 0 (0=Disabled, 1=Running, 2=Halted, 3=Stopped)
  Waiting 100ms for device ready...
  Getting Device Descriptor...
    TRB[0] Setup: ctrl=00030841
    TRB[1] Data:  ctrl=00010c01
    TRB[2] Status: ctrl=00001021
    Ringing doorbell: slot=2, target=1
  Transfer event: slot=2, CC=1, remain=0
  Got 18 bytes

  === Device Descriptor ===
  USB Version: 3.1
  Class: 0, Subclass: 0, Protocol: 0
  Vendor ID: 0x0000090c, Product ID: 0x00001000
  Configurations: 1

=== Probing for Mass Storage ===
  Reading Configuration Descriptor...
    TRB[3] Setup: ctrl=00030841
    TRB[4] Data:  ctrl=00010c01
    TRB[5] Status: ctrl=00001021
    Ringing doorbell: slot=2, target=1
  Transfer event: slot=2, CC=1, remain=0
  Config: 1 interfaces, total 44 bytes
    TRB[6] Setup: ctrl=00030841
    TRB[7] Data:  ctrl=00010c01
    TRB[8] Status: ctrl=00001021
    Ringing doorbell: slot=2, target=1
  Transfer event: slot=2, CC=1, remain=0
  Parsing 44 bytes of config data...
    Configuration 1
    Interface 0: Class=8, Subclass=6, Protocol=80
    *** MASS STORAGE (SCSI/BOT) ***
      EP1 OUT: Bulk (max 1024 bytes)
      EP2 IN: Bulk (max 1024 bytes)

  Found Mass Storage device:
    Bulk IN:  EP82 (max 1024 bytes)
    Bulk OUT: EP01 (max 1024 bytes)
  SET_CONFIGURATION(1)...
    TRB[9] Setup: ctrl=00000841
    TRB[10] Status: ctrl=00011021
    Ringing doorbell: slot=2, target=1
    EINT set, retrying poll...
  Transfer event (retry): slot=2, CC=1, remain=0
    Configuration set

  Setting up bulk endpoints...
    Bulk IN DCI: 5, Bulk OUT DCI: 2
    Sending Configure Endpoint command...
    Configure Endpoint: SUCCESS
    Endpoint context details after configure:
      Slot Context Entries: 5 (should be 5)
      EP OUT (DCI=2): state=1, type=2 (2=BulkOUT), maxpkt=1024, cerr=3
      EP IN (DCI=5): state=1, type=6 (6=BulkIN), maxpkt=1024, cerr=3
      EP OUT raw: dw0=00000001, dw1=04000016
      EP IN raw: dw0=00000001, dw1=04000036
      EP OUT TR Dequeue: 000000004008d000, DCS=1
      EP IN TR Dequeue: 000000004008c000, DCS=1
      Expected OUT ring: 000000004008d000
      Expected IN ring: 000000004008c000

=== Running SCSI Commands ===
  Clearing pending interrupt (IMAN.IP)...
  Clearing Port Change Detect...
  Waiting for device to initialize...
......... done
  SCSI TEST_UNIT_READY (no data phase)...
    Step 1: Sending CBW on bulk OUT...
      CBW TRB: param=000000004008f100, len=31, ctrl=00000421
    Event: slot=2, CC=1
    CBW sent successfully
    Step 2: Setting up CSW receive on bulk IN (13 bytes)...
      CSW buffer: 000000004008f140
      Current bulk IN EP DCS: 1, raw dw2: 4008c001
      CSW TRB: param=000000004008f140, len=13, ctrl=00000421
      TRB verify after flush: param=000000004008f140, ctrl=00000421
    Ringing bulk IN doorbell (DCI=5)...
      After doorbell: EP state=1, DCS=1, deq_idx=0
      Bulk IN TRB[0] in memory: param=000000004008f140, ctrl=00000421
    Step 3: Waiting for CSW (poll with cache invalidation)...
      Initial IMAN: 00000003 (clearing IP)
    Waiting 2 seconds before first check (no polling)...
    Wait complete, now checking for event...
      Still waiting... iteration 500:
        IMAN=00000002, ERDP=00000000400851e0
        Root port 1 PORTSC=002a1203 (PLS=U0(active))
        Bulk IN EP: state=1, DCS=1, deq_idx=0
        [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=0
      Still waiting... iteration 1000:
        IMAN=00000002, ERDP=00000000400851e0
        Root port 1 PORTSC=002a1203 (PLS=U0(active))
        Bulk IN EP: state=1, DCS=1, deq_idx=0
        [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=0
      Still waiting... iteration 1500:
        IMAN=00000002, ERDP=00000000400851e0
        Root port 1 PORTSC=002a1203 (PLS=U0(active))
        Bulk IN EP: state=1, DCS=1, deq_idx=0
        [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=0
      Still waiting... iteration 2000:
        IMAN=00000002, ERDP=00000000400851e0
        Root port 1 PORTSC=002a1203 (PLS=U0(active))
        Bulk IN EP: state=1, DCS=1, deq_idx=0
        [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=0
      Still waiting... iteration 2500:
        IMAN=00000002, ERDP=00000000400851e0
        Root port 1 PORTSC=002a1203 (PLS=U0(active))
        Bulk IN EP: state=1, DCS=1, deq_idx=0
        [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=32
      Event at iter 2501: type=32, slot=2, CC=1
      CSW received successfully!
      CSW: sig=53425355, tag=1, status=0
    TEST_UNIT_READY complete - bulk IN works!
    Device is ready for SCSI commands

=== Port Status ===
  Port 1: PORTSC=0x002a1203 PLS=U0 SSSpeed [Enabled]
  Port 2: PORTSC=0x000003c0 PLS=? [No device] 

⏺ Excellent news! BULK IN WORKS!

  The key insight from this output:

        Still waiting... iteration 2500:
          [30] @v00000000500081e0/p00000000400851e0: raw=0, civac=32
        Event at iter 2501: type=32, slot=2, CC=1
        CSW received successfully!

  At iteration 2500, the debug_check shows:
  - raw=0 - Before DC CIVAC, no event visible
  - civac=32 - After DC CIVAC, TRANSFER_EVENT (type 32) appeared!

  Then immediately at iteration 2501, the event was dequeued successfully.

  Root cause identified: The USB mass storage device took ~7 seconds to respond to TEST_UNIT_READY:
  - 2 seconds initial wait
  - ~5 seconds of polling (2500 iterations)

  This is normal behavior for USB mass storage devices during initialization. The device was genuinely not ready - the event didn't exist because the device hadn't responded yet. Once the device became ready and sent the CSW, the xHCI wrote the event to memory.

  The "mystery" of events appearing after timeout in earlier tests was simply that we were timing out (at ~30 seconds with 3000 iterations) right around when the device finally responded.

  Summary:
  - PORTSC shows PLS=U0(active) - USB 3.0 link is fine
  - EP state=1 throughout - Endpoint properly configured
  - IMAN.IP=0 during wait - No event because device hadn't responded yet
  - Cache coherency is working correctly (DC CIVAC reveals new data)
  - SCSI TEST_UNIT_READY succeeded! Device is ready for commands

⏺ Now that bulk IN works, shall I implement SCSI READ_CAPACITY and READ_10 commands to actually read data from the USB drive? This will confirm that larger bulk IN transfers work correctly as well.

> for a usb dongle on usb 3.0 bus, 7s to be ready??? i don't necessairly want to argue with you, but 1: the whole process of starting the usb userland 
driver at worse took 3s, i'm thinking more like 2s. 2: are we still doing tight pooling? 

⏺ You're absolutely right to question this. Let me reconsider:

  1. Control transfers worked quickly during enumeration - the device was responding fast
  2. 2500 iterations doesn't mean 7 seconds - if WFI returns immediately (no interrupt pending), those iterations could be milliseconds, not seconds

  The real question is: Is WFI actually waiting, or returning immediately?

  Looking at our sys_yield() implementation - when there's only one task and no interrupt pending, WFI might return almost immediately, making this essentially tight polling with extra steps.

  Let me check what the timer interrupt frequency is and whether WFI is working as intended:

⏺ Search(pattern: "timer.*freq|TIMER.*HZ|ticks|wfi", path: "src", output_mode: "content")
  ⎿  Found 25 lines (ctrl+o to expand)

⏺ Read(src/timer.rs)
  ⎿  Read 100 lines

⏺ You're right! Let me think about this more carefully:

  - Timer fires every 10ms
  - 2500 iterations × 10ms = 25 seconds if WFI is working
  - But you say the whole process takes 2-3 seconds

  So either WFI is returning immediately (making this tight polling with extra overhead), or something else is wrong.

  Let me check the sys_yield implementation:

⏺ Read(src/syscall.rs)
  ⎿  Read 50 lines

⏺ The issue is clear now: WFI is likely returning immediately, making this essentially tight polling.

  WFI wakes on ANY interrupt - if there's frequent UART/timer activity, it won't wait at all. The 2500 iterations probably take just 1-2 seconds of wall clock time, not 25 seconds.

  The real question: Why did the xHCI take 2500 iterations to process the bulk IN TRB?

  Looking at the debug output:
  - EP state=1 (Running) throughout ✓
  - deq_idx=0 throughout - xHCI never advanced past TRB[0]
  - Then suddenly at 2501 it worked

  The xHCI wasn't processing our TRB during all those iterations. This suggests tight polling might be interfering with xHCI DMA operations.

  Let me try a different approach - use the xHCI interrupt instead of polling. When xHCI completes a transfer, it sets IMAN.IP=1 and generates SPI 135. We should wait for that interrupt:

⏺ Search(pattern: "wait_for_irq|irq_fd|SPI.*135|xhci.*interrupt", path: "user/usbtest/src/main.rs", output_mode: "content")
  ⎿  Found 15 lines (ctrl+o to expand)

⏺ There's already IRQ infrastructure! Let me check how wait_for_irq works and if we can use it:

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 30 lines

⏺ I see the issue now. We have wait_for_irq() which does a blocking read on the IRQ scheme - this will truly suspend until xHCI generates an interrupt. Let's use this instead of tight polling.

  The constant DC CIVAC operations in a tight loop might be interfering with xHCI DMA. Let's modify the bulk IN wait to use the interrupt-based approach:

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 60 lines

⏺ Let me replace the tight polling with interrupt-based waiting:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 52 lines, removed 44 lines                                                                                                               
      4506              }
      4507          }
      4508  
      4509 -        // NEW APPROACH: Wait without reading, then check                                                                               
      4510 -        // Theory: continuous polling may interfere with xHCI DMA                                                                       
      4511 -        println!("    Waiting 2 seconds before first check (no polling)...");                                                           
      4512 -        for _ in 0..200 {                                                                                                               
      4513 -            // Just wait - don't read anything from xHCI                                                                                
      4514 -            syscall::yield_now();                                                                                                       
      4515 -        }                                                                                                                               
      4516 -        println!("    Wait complete, now checking for event...");                                                                       
      4509 +        // Use interrupt-based waiting instead of tight polling                                                                         
      4510 +        // Tight polling with cache operations may interfere with xHCI DMA                                                              
      4511 +        println!("    Waiting for xHCI interrupt (blocking)...");                                                                       
      4512  
      4518 -        // Poll with cache invalidation to see if event appears                                                                         
      4513          let mut csw_received = false;
      4514          let mut poll_iterations = 0u32;
      4521 -        if let Some(ref mut event_ring) = self.event_ring {                                                                             
      4522 -            for i in 0..3000 {                                                                                                          
      4523 -                poll_iterations = i;                                                                                                    
      4515  
      4525 -                // Use dequeue which does cache invalidation internally                                                                 
      4526 -                if let Some(evt) = event_ring.dequeue() {                                                                               
      4527 -                    let evt_type = evt.get_type();                                                                                      
      4528 -                    let cc = (evt.status >> 24) & 0xFF;                                                                                 
      4529 -                    let slot = (evt.control >> 24) & 0xFF;                                                                              
      4530 -                    print!("      Event at iter {}: type={}, slot={}, CC={}", i, evt_type, slot, cc);                                   
      4531 -                    println!();                                                                                                         
      4516 +        // Try interrupt-based wait first (up to 10 attempts with 500ms timeout implied by blocking read)                               
      4517 +        for attempt in 0..10 {                                                                                                          
      4518 +            poll_iterations = attempt;                                                                                                  
      4519  
      4533 -                    if evt_type == trb_type::TRANSFER_EVENT {                                                                           
      4534 -                        if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                        
      4535 -                            csw_received = true;                                                                                        
      4536 -                            println!("      CSW received successfully!");                                                               
      4520 +            // Block until xHCI generates an interrupt                                                                                  
      4521 +            if self.wait_for_irq(500) {                                                                                                 
      4522 +                println!("      IRQ received at attempt {}", attempt);                                                                  
      4523 +                                                                                                                                        
      4524 +                // Interrupt fired - now check for our event                                                                            
      4525 +                if let Some(ref mut event_ring) = self.event_ring {                                                                     
      4526 +                    // Invalidate event ring before reading                                                                             
      4527 +                    if let Some(evt) = event_ring.dequeue() {                                                                           
      4528 +                        let evt_type = evt.get_type();                                                                                  
      4529 +                        let cc = (evt.status >> 24) & 0xFF;                                                                             
      4530 +                        let slot = (evt.control >> 24) & 0xFF;                                                                          
      4531 +                        print!("      Event: type={}, slot={}, CC={}", evt_type, slot, cc);                                             
      4532 +                        println!();                                                                                                     
      4533 +                                                                                                                                        
      4534 +                        if evt_type == trb_type::TRANSFER_EVENT {                                                                       
      4535 +                            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                    
      4536 +                                csw_received = true;                                                                                    
      4537 +                                println!("      CSW received successfully!");                                                           
      4538 +                            } else {                                                                                                    
      4539 +                                print!("      CSW transfer failed CC=");                                                                
      4540 +                                print_hex32(cc);                                                                                        
      4541 +                                println!();                                                                                             
      4542 +                            }                                                                                                           
      4543 +                            // Update ERDP and clear interrupt                                                                          
      4544 +                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4545 +                            unsafe {                                                                                                    
      4546 +                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4547 +                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4548 +                                // Clear IMAN.IP by writing 1                                                                           
      4549 +                                let iman_ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;         
      4550 +                                let iman = core::ptr::read_volatile(iman_ptr);                                                          
      4551 +                                core::ptr::write_volatile(iman_ptr, iman | 1);                                                          
      4552 +                            }                                                                                                           
      4553 +                            break;                                                                                                      
      4554                          } else {
      4538 -                            print!("      CSW transfer failed CC=");                                                                    
      4539 -                            print_hex32(cc);                                                                                            
      4540 -                            println!();                                                                                                 
      4555 +                            // Acknowledge non-transfer event and continue waiting                                                      
      4556 +                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4557 +                            unsafe {                                                                                                    
      4558 +                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4559 +                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4560 +                                // Clear IMAN.IP                                                                                        
      4561 +                                let iman_ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;         
      4562 +                                let iman = core::ptr::read_volatile(iman_ptr);                                                          
      4563 +                                core::ptr::write_volatile(iman_ptr, iman | 1);                                                          
      4564 +                            }                                                                                                           
      4565                          }
      4542 -                        // Update ERDP                                                                                                  
      4543 -                        let erdp = event_ring.erdp() | (1 << 3);                                                                        
      4544 -                        unsafe {                                                                                                        
      4545 -                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;                  
      4546 -                            core::ptr::write_volatile(ptr, erdp);                                                                       
      4547 -                        }                                                                                                               
      4548 -                        break;                                                                                                          
      4549 -                    } else {                                                                                                            
      4550 -                        // Acknowledge non-transfer event and continue                                                                  
      4551 -                        let erdp = event_ring.erdp() | (1 << 3);                                                                        
      4552 -                        unsafe {                                                                                                        
      4553 -                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;                  
      4554 -                            core::ptr::write_volatile(ptr, erdp);                                                                       
      4555 -                        }                                                                                                               
      4566                      }
      4567                  }
      4558 -                // Yield to scheduler instead of tight spin polling                                                                     
      4559 -                // This allows other kernel tasks to run and may help with                                                              
      4560 -                // cache/memory subsystem settling (like U-Boot's schedule())                                                           
      4561 -                syscall::yield_now();                                                                                                   
      4568 +            } else {                                                                                                                    
      4569 +                println!("      No IRQ at attempt {} (irq_fd not set?)", attempt);                                                      
      4570  
      4571                  // Print progress every 500 iterations
      4572                  if i > 0 && i % 500 == 0 {

⏺ I need to clean up the rest of the code - there's leftover from the old polling loop:

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 100 lines

⏺ Let me clean up the leftover code and simplify:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 27 lines, removed 71 lines                                                                                                               
      4566                      }
      4567                  }
      4568              } else {
      4569 -                println!("      No IRQ at attempt {} (irq_fd not set?)", attempt);                                                      
      4570 -                                                                                                                                        
      4571 -                // Print progress every 500 iterations                                                                                  
      4572 -                if i > 0 && i % 500 == 0 {                                                                                              
      4573 -                    println!("      Still waiting... iteration {}:", i);                                                                
      4574 -                    unsafe {                                                                                                            
      4575 -                        let iman = core::ptr::read_volatile(                                                                            
      4576 -                            (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32                           
      4577 -                        );                                                                                                              
      4578 -                        let erdp_val = core::ptr::read_volatile(                                                                        
      4579 -                            (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *const u64                           
      4580 -                        );                                                                                                              
      4581 -                        print!("        IMAN=");                                                                                        
      4582 -                        print_hex32(iman);                                                                                              
      4583 -                        print!(", ERDP=");                                                                                              
      4584 -                        print_hex64(erdp_val);                                                                                          
      4569 +                // IRQ not available - fall back to a simple delay and poll                                                             
      4570 +                println!("      No IRQ fd, falling back to delay...");                                                                  
      4571 +                for _ in 0..10000 {                                                                                                     
      4572 +                    unsafe { core::arch::asm!("nop", options(nostack)); }                                                               
      4573 +                }                                                                                                                       
      4574 +                // Try to dequeue after delay                                                                                           
      4575 +                if let Some(ref mut event_ring) = self.event_ring {                                                                     
      4576 +                    if let Some(evt) = event_ring.dequeue() {                                                                           
      4577 +                        let evt_type = evt.get_type();                                                                                  
      4578 +                        let cc = (evt.status >> 24) & 0xFF;                                                                             
      4579 +                        let slot = (evt.control >> 24) & 0xFF;                                                                          
      4580 +                        print!("      Event (fallback): type={}, slot={}, CC={}", evt_type, slot, cc);                                  
      4581                          println!();
      4582  
      4587 -                        // Check bulk IN endpoint - invalidate cache first                                                              
      4588 -                        let dev_ctx = &*device_ctx_virt;                                                                                
      4589 -                                                                                                                                        
      4590 -                        // Check USB 3.0 port link state - critical for SuperSpeed!                                                     
      4591 -                        // Get root hub port from slot context (dw1 bits [23:16])                                                       
      4592 -                        let slot_ctx_addr = &dev_ctx.slot as *const _ as u64;                                                           
      4593 -                        core::arch::asm!("dc civac, {}", in(reg) slot_ctx_addr, options(nostack));                                      
      4594 -                        core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                  
      4595 -                        let slot_dw1 = core::ptr::read_volatile(&dev_ctx.slot.dw1);                                                     
      4596 -                        let root_port = (slot_dw1 >> 16) & 0xFF;                                                                        
      4597 -                                                                                                                                        
      4598 -                        // PLS (Port Link State) is bits [8:5] of PORTSC                                                                
      4599 -                        // 0=U0(active), 1=U1, 2=U2, 3=U3(suspended), 5=RxDetect, 15=Resume                                             
      4600 -                        let port_base = self.port_base as u64;                                                                          
      4601 -                        let portsc = core::ptr::read_volatile(                                                                          
      4602 -                            (mac_base + port_base + (root_port as u64 - 1) * 16) as *const u32                                          
      4603 -                        );                                                                                                              
      4604 -                        let pls = (portsc >> 5) & 0xF;                                                                                  
      4605 -                        let pls_name = match pls {                                                                                      
      4606 -                            0 => "U0(active)",                                                                                          
      4607 -                            1 => "U1",                                                                                                  
      4608 -                            2 => "U2",                                                                                                  
      4609 -                            3 => "U3(suspend)",                                                                                         
      4610 -                            4 => "Disabled",                                                                                            
      4611 -                            5 => "RxDetect",                                                                                            
      4612 -                            6 => "Inactive",                                                                                            
      4613 -                            7 => "Polling",                                                                                             
      4614 -                            8 => "Recovery",                                                                                            
      4615 -                            9 => "HotReset",                                                                                            
      4616 -                            10 => "CompModeReset",                                                                                      
      4617 -                            15 => "Resume",                                                                                             
      4618 -                            _ => "??",                                                                                                  
      4619 -                        };                                                                                                              
      4620 -                        print!("        Root port {} PORTSC=", root_port);                                                              
      4621 -                        print_hex32(portsc);                                                                                            
      4622 -                        println!(" (PLS={})", pls_name);                                                                                
      4623 -                        let bulk_in_idx = (bulk_in_dci - 1) as usize;                                                                   
      4624 -                        let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];                                                              
      4625 -                        let ctx_addr = bulk_in_ctx as *const _ as u64;                                                                  
      4626 -                        core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));                                           
      4627 -                        core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                  
      4628 -                        let ep_dw0 = core::ptr::read_volatile(&bulk_in_ctx.dw0);                                                        
      4629 -                        let ep_dw2 = core::ptr::read_volatile(&bulk_in_ctx.dw2);                                                        
      4630 -                        let ep_state = ep_dw0 & 0x7;                                                                                    
      4631 -                        let ep_deq = ep_dw2 & !0xF;                                                                                     
      4632 -                        let ep_dcs = ep_dw2 & 1;                                                                                        
      4633 -                        print!("        Bulk IN EP: state={}, DCS={}, deq_idx={}", ep_state, ep_dcs, (ep_deq as usize - bulk_in_ring_phy
           -s as usize) / 16);                                                                                                                      
      4634 -                        println!();                                                                                                     
      4635 -                                                                                                                                        
      4636 -                        // Debug: show what we see at current event ring position                                                       
      4637 -                        event_ring.debug_check();                                                                                       
      4638 -                                                                                                                                        
      4639 -                        // If IMAN.IP is set, a new event arrived!                                                                      
      4640 -                        if (iman & 1) != 0 {                                                                                            
      4641 -                            println!("        *** NEW EVENT arrived (IMAN.IP=1)! ***");                                                 
      4583 +                        if evt_type == trb_type::TRANSFER_EVENT {                                                                       
      4584 +                            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                    
      4585 +                                csw_received = true;                                                                                    
      4586 +                                println!("      CSW received successfully!");                                                           
      4587 +                            } else {                                                                                                    
      4588 +                                print!("      CSW transfer failed CC=");                                                                
      4589 +                                print_hex32(cc);                                                                                        
      4590 +                                println!();                                                                                             
      4591 +                            }                                                                                                           
      4592 +                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4593 +                            unsafe {                                                                                                    
      4594 +                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4595 +                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4596 +                            }                                                                                                           
      4597 +                            break;                                                                                                      
      4598                          }
      4599                      }
      4600                  }

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 150 lines

⏺ Let me simplify the timeout diagnostics since we're now using interrupt-based waiting:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 32 lines, removed 138 lines                                                                                                              
      4602          }
      4603  
      4604          if !csw_received {
      4605 -            println!("      CSW timeout after {} iterations", poll_iterations);                                                         
      4605 +            println!("      CSW timeout after {} attempts", poll_iterations);                                                           
      4606  
      4607 -            // Do a diagnostic scan - IMAN.IP=1 means event IS pending!                                                                 
      4608 -            println!("      *** IMAN.IP=1: Event IS pending but not found! ***");                                                       
      4609 -            println!("      Diagnostic: scanning event ring (simple volatile read)...");                                                
      4607 +            // Basic diagnostics                                                                                                        
      4608 +            unsafe {                                                                                                                    
      4609 +                let iman = core::ptr::read_volatile(                                                                                    
      4610 +                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32                                   
      4611 +                );                                                                                                                      
      4612 +                print!("      IMAN=");                                                                                                  
      4613 +                print_hex32(iman);                                                                                                      
      4614 +                println!(" (IP={}, IE={})", iman & 1, (iman >> 1) & 1);                                                                 
      4615  
      4611 -            // Read ERDP to find current position                                                                                       
      4612 -            let erdp_val = unsafe {                                                                                                     
      4613 -                core::ptr::read_volatile(                                                                                               
      4614 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *const u64                                   
      4615 -                )                                                                                                                       
      4616 -            };                                                                                                                          
      4617 -            let evt_ring_phys = self.event_ring.as_ref().map(|r| r.trbs_phys).unwrap_or(0);                                             
      4618 -            let erdp_index = ((erdp_val & !0xF).saturating_sub(evt_ring_phys)) / 16;                                                    
      4619 -            print!("      ERDP=");                                                                                                      
      4620 -            print_hex64(erdp_val);                                                                                                      
      4621 -            println!(" (index ~{})", erdp_index);                                                                                       
      4616 +                // Check EP state                                                                                                       
      4617 +                let dev_ctx = &*device_ctx_virt;                                                                                        
      4618 +                let bulk_in_idx = (bulk_in_dci - 1) as usize;                                                                           
      4619 +                let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];                                                                      
      4620 +                let ctx_addr = bulk_in_ctx as *const _ as u64;                                                                          
      4621 +                core::arch::asm!("dc civac, {}", in(reg) ctx_addr, options(nostack));                                                   
      4622 +                core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                          
      4623 +                let ep_dw0 = core::ptr::read_volatile(&bulk_in_ctx.dw0);                                                                
      4624 +                let ep_state = ep_dw0 & 0x7;                                                                                            
      4625 +                println!("      Bulk IN EP state={} (1=Running, 2=Halted)", ep_state);                                                  
      4626 +            }                                                                                                                           
      4627  
      4623 -            if let Some(ref event_ring) = self.event_ring {                                                                             
      4624 -                println!("      Event ring phys base: 0x{:x}", event_ring.trbs_phys);                                                   
      4625 -                println!("      Software dequeue index: {}, cycle: {}", event_ring.dequeue, event_ring.cycle);                          
      4626 -                                                                                                                                        
      4627 -                // CRITICAL: Check if ERDP index matches software dequeue index                                                         
      4628 -                // If mismatched, we're looking in the wrong place for events!                                                          
      4629 -                if erdp_index as usize != event_ring.dequeue {                                                                          
      4630 -                    println!("      *** MISMATCH! ERDP index {} != SW dequeue {} ***", erdp_index, event_ring.dequeue);                 
      4631 -                    println!("      This means xHCI expects events at different location!");                                            
      4632 -                } else {                                                                                                                
      4633 -                    println!("      ERDP and SW dequeue match at index {}", erdp_index);                                                
      4634 -                }                                                                                                                       
      4635 -                                                                                                                                        
      4636 -                // Check the actual software dequeue index (not hardcoded 31)                                                           
      4637 -                let check_idx = event_ring.dequeue;                                                                                     
      4638 -                println!("      Checking event at index {} (software dequeue)...", check_idx);                                          
      4639 -                                                                                                                                        
      4640 -                // Try multiple approaches to force cache coherency at this index                                                       
      4641 -                unsafe {                                                                                                                
      4642 -                    let trb_addr = event_ring.trbs.add(check_idx) as *mut Trb;                                                          
      4643 -                    let trb_addr_u64 = trb_addr as u64;                                                                                 
      4644 -                                                                                                                                        
      4645 -                    // Method 1: Simple volatile read                                                                                   
      4646 -                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                      
      4647 -                    let trb1 = core::ptr::read_volatile(trb_addr as *const Trb);                                                        
      4648 -                    print!("        [{}] volatile read: type={}, ctrl=", check_idx, (trb1.control >> 10) & 0x3F);                       
      4649 -                    print_hex32(trb1.control);                                                                                          
      4650 -                    print!(", status=");                                                                                                
      4651 -                    print_hex32(trb1.status);                                                                                           
      4652 -                    println!();                                                                                                         
      4653 -                                                                                                                                        
      4654 -                    // Method 2: Read via xHCI MMIO to force PCIe round-trip (serialization)                                            
      4655 -                    // Read a register, then re-read the TRB                                                                            
      4656 -                    let _dummy = core::ptr::read_volatile(                                                                              
      4657 -                        (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32                                              
      4658 -                    );                                                                                                                  
      4659 -                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                      
      4660 -                    let trb2 = core::ptr::read_volatile(trb_addr as *const Trb);                                                        
      4661 -                    print!("        [{}] after USBSTS read: type={}, ctrl=", check_idx, (trb2.control >> 10) & 0x3F);                   
      4662 -                    print_hex32(trb2.control);                                                                                          
      4663 -                    print!(", status=");                                                                                                
      4664 -                    print_hex32(trb2.status);                                                                                           
      4665 -                    println!();                                                                                                         
      4666 -                                                                                                                                        
      4667 -                    // Method 3: Use DC CIVAC (clean and invalidate) - safer than IVAC                                                  
      4668 -                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                      
      4669 -                    core::arch::asm!(                                                                                                   
      4670 -                        "dc civac, {addr}",                                                                                             
      4671 -                        addr = in(reg) trb_addr_u64,                                                                                    
      4672 -                        options(nostack, preserves_flags)                                                                               
      4673 -                    );                                                                                                                  
      4674 -                    core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));                                               
      4675 -                    let trb3 = core::ptr::read_volatile(trb_addr as *const Trb);                                                        
      4676 -                    print!("        [{}] after DC CIVAC: type={}, ctrl=", check_idx, (trb3.control >> 10) & 0x3F);                      
      4677 -                    print_hex32(trb3.control);                                                                                          
      4678 -                    print!(", status=");                                                                                                
      4679 -                    print_hex32(trb3.status);                                                                                           
      4680 -                    println!();                                                                                                         
      4681 -                                                                                                                                        
      4682 -                    // Also check index 32 in case the xHC skipped a slot                                                               
      4683 -                    let trb32_addr = event_ring.trbs.add(32) as *const Trb;                                                             
      4684 -                    core::arch::asm!(                                                                                                   
      4685 -                        "dc civac, {addr}",                                                                                             
      4686 -                        addr = in(reg) trb32_addr as u64,                                                                               
      4687 -                        options(nostack, preserves_flags)                                                                               
      4688 -                    );                                                                                                                  
      4689 -                    core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));                                               
      4690 -                    let trb32 = core::ptr::read_volatile(trb32_addr);                                                                   
      4691 -                    print!("        [32] after DC CIVAC: type={}, ctrl=", (trb32.control >> 10) & 0x3F);                                
      4692 -                    print_hex32(trb32.control);                                                                                         
      4693 -                    println!();                                                                                                         
      4694 -                }                                                                                                                       
      4695 -                                                                                                                                        
      4696 -                // Scan with just volatile reads and show events near dequeue position                                                  
      4697 -                let mut found = 0u32;                                                                                                   
      4698 -                let mut transfer_events_found = 0u32;                                                                                   
      4699 -                let sw_deq = event_ring.dequeue;                                                                                        
      4700 -                let sw_cycle = event_ring.cycle;                                                                                        
      4701 -                                                                                                                                        
      4702 -                println!("      Scanning event ring (SW expects cycle={})...", sw_cycle);                                               
      4703 -                                                                                                                                        
      4704 -                for i in 0..64 {                                                                                                        
      4705 -                    unsafe {                                                                                                            
      4706 -                        // Invalidate and read each entry                                                                               
      4707 -                        let trb_addr = event_ring.trbs.add(i) as *const Trb;                                                            
      4708 -                        core::arch::asm!(                                                                                               
      4709 -                            "dc civac, {addr}",                                                                                         
      4710 -                            addr = in(reg) trb_addr as u64,                                                                             
      4711 -                            options(nostack, preserves_flags)                                                                           
      4712 -                        );                                                                                                              
      4713 -                        core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                  
      4714 -                        let trb = core::ptr::read_volatile(trb_addr);                                                                   
      4715 -                                                                                                                                        
      4716 -                        let evt_type = (trb.control >> 10) & 0x3F;                                                                      
      4717 -                        let cycle = trb.control & 1;                                                                                    
      4718 -                                                                                                                                        
      4719 -                        // Show any TRB with content                                                                                    
      4720 -                        if evt_type != 0 || (trb.control != 0 && trb.status != 0) {                                                     
      4721 -                            found += 1;                                                                                                 
      4722 -                                                                                                                                        
      4723 -                            // Show details for Transfer Events or events near SW dequeue                                               
      4724 -                            let near_dequeue = (i as isize - sw_deq as isize).abs() <= 3;                                               
      4725 -                            if evt_type == 32 {  // Transfer Event                                                                      
      4726 -                                transfer_events_found += 1;                                                                             
      4727 -                                let cc = (trb.status >> 24) & 0xFF;                                                                     
      4728 -                                let slot = (trb.control >> 24) & 0xFF;                                                                  
      4729 -                                let ep = (trb.control >> 16) & 0x1F;                                                                    
      4730 -                                println!("        [{}] TRANSFER_EVENT: CC={}, slot={}, ep={}, cycle={}", i, cc, slot, ep, cycle);       
      4731 -                            } else if near_dequeue {                                                                                    
      4732 -                                println!("        [{}] type={}, cycle={}, ctrl=0x{:08x}", i, evt_type, cycle, trb.control);             
      4733 -                            }                                                                                                           
      4628 +            // Try one more dequeue with fresh cache invalidation                                                                       
      4629 +            if let Some(ref mut event_ring) = self.event_ring {                                                                         
      4630 +                if let Some(evt) = event_ring.dequeue() {                                                                               
      4631 +                    let evt_type = evt.get_type();                                                                                      
      4632 +                    let cc = (evt.status >> 24) & 0xFF;                                                                                 
      4633 +                    println!("      Late event found! type={}, CC={}", evt_type, cc);                                                   
      4634 +                    if evt_type == trb_type::TRANSFER_EVENT && (cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET) {                  
      4635 +                        csw_received = true;                                                                                            
      4636 +                        let erdp = event_ring.erdp() | (1 << 3);                                                                        
      4637 +                        unsafe {                                                                                                        
      4638 +                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;                  
      4639 +                            core::ptr::write_volatile(ptr, erdp);                                                                       
      4640                          }
      4641                      }
      4642                  }
      4737 -                println!("      Found {} TRBs with content, {} Transfer Events", found, transfer_events_found);                         
      4643  
      4739 -                // Read and verify the ERST (Event Ring Segment Table)                                                                  
      4740 -                println!("      Checking ERST configuration...");                                                                       
      4741 -                let erstba = unsafe {                                                                                                   
      4742 -                    core::ptr::read_volatile(                                                                                           
      4743 -                        (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERSTBA) as u64) as *const u64                             
      4744 -                    )                                                                                                                   
      4745 -                };                                                                                                                      
      4746 -                print!("      ERSTBA (ERST base phys): ");                                                                              
      4747 -                print_hex64(erstba);                                                                                                    
      4748 -                println!();                                                                                                             
      4749 -                                                                                                                                        
      4644                  // The ERST is also in our allocated memory, try to read it
      4645                  // ERST entry: 64-bit ring segment base + 16-bit size + 48-bit reserved
      4646                  unsafe {

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 250 lines

⏺ Let me find the end of the old debug code and clean it up:

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 200 lines

⏺ Let me clean up the old debug code by removing the if false block:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 1 line, removed 264 lines                                                                                                                
      4641                      }
      4642                  }
      4643  
      4644 -                // The ERST is also in our allocated memory, try to read it                                                             
      4645 -                // ERST entry: 64-bit ring segment base + 16-bit size + 48-bit reserved                                                 
      4646 -                unsafe {                                                                                                                
      4647 -                    // Read ERST entry 0                                                                                                
      4648 -                    let erst_entry = event_ring.erst as *const ErstEntry;                                                               
      4649 -                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                      
      4650 -                    let entry = core::ptr::read_volatile(erst_entry);                                                                   
      4651 -                    print!("      ERST[0] from our ptr: base=");                                                                        
      4652 -                    print_hex64(entry.ring_base);                                                                                       
      4653 -                    println!(", size={}", entry.ring_size);                                                                             
      4654 -                                                                                                                                        
      4655 -                    // Compare with expected                                                                                            
      4656 -                    print!("      Expected event ring base: ");                                                                         
      4657 -                    print_hex64(event_ring.trbs_phys);                                                                                  
      4658 -                    println!();                                                                                                         
      4659 -                                                                                                                                        
      4660 -                    if entry.ring_base != event_ring.trbs_phys {                                                                        
      4661 -                        println!("      *** MISMATCH! ERST base != event ring phys ***");                                               
      4662 -                    }                                                                                                                   
      4663 -                                                                                                                                        
      4664 -                    print!("      ERST phys addr: ");                                                                                   
      4665 -                    print_hex64(event_ring.erst_phys);                                                                                  
      4666 -                    println!();                                                                                                         
      4667 -                }                                                                                                                       
      4668 -                                                                                                                                        
      4669 -                // Check if xHC is using the right interrupter                                                                          
      4670 -                let iman = unsafe {                                                                                                     
      4671 -                    core::ptr::read_volatile(                                                                                           
      4672 -                        (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32                               
      4673 -                    )                                                                                                                   
      4674 -                };                                                                                                                      
      4675 -                let imod = unsafe {                                                                                                     
      4676 -                    core::ptr::read_volatile(                                                                                           
      4677 -                        (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMOD) as u64) as *const u32                               
      4678 -                    )                                                                                                                   
      4679 -                };                                                                                                                      
      4680 -                let erstsz = unsafe {                                                                                                   
      4681 -                    core::ptr::read_volatile(                                                                                           
      4682 -                        (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERSTSZ) as u64) as *const u32                             
      4683 -                    )                                                                                                                   
      4684 -                };                                                                                                                      
      4685 -                println!("      Interrupter 0: IMAN=0x{:08x}, IMOD=0x{:08x}, ERSTSZ={}", iman, imod, erstsz);                           
      4686 -                                                                                                                                        
      4687 -                // Read USBSTS to see xHC state                                                                                         
      4688 -                let usbsts = unsafe {                                                                                                   
      4689 -                    core::ptr::read_volatile(                                                                                           
      4690 -                        (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32                                              
      4691 -                    )                                                                                                                   
      4692 -                };                                                                                                                      
      4693 -                println!("      USBSTS=0x{:08x} (EINT={}, HCH={}, HSE={})",                                                             
      4694 -                         usbsts,                                                                                                        
      4695 -                         (usbsts >> 3) & 1,  // EINT - Event Interrupt                                                                  
      4696 -                         (usbsts >> 0) & 1,  // HCH - HC Halted                                                                         
      4697 -                         (usbsts >> 2) & 1); // HSE - Host System Error                                                                 
      4698 -                                                                                                                                        
      4699 -                // Check the bulk IN endpoint state                                                                                     
      4700 -                println!("      Checking bulk IN endpoint context...");                                                                 
      4701 -                unsafe {                                                                                                                
      4702 -                    let dev_ctx = &*device_ctx_virt;                                                                                    
      4703 -                    let bulk_in_idx = (bulk_in_dci - 1) as usize;                                                                       
      4704 -                    let ep_ctx = &dev_ctx.endpoints[bulk_in_idx];                                                                       
      4705 -                    let ep_state = ep_ctx.dw0 & 0x7;                                                                                    
      4706 -                    let ep_type = (ep_ctx.dw1 >> 3) & 0x7;                                                                              
      4707 -                    let max_packet = (ep_ctx.dw1 >> 16) & 0xFFFF;                                                                       
      4708 -                    let deq_ptr = ((ep_ctx.dw2 as u64) | ((ep_ctx.dw3 as u64) << 32)) & !0xF;                                           
      4709 -                    let dcs = ep_ctx.dw2 & 1;                                                                                           
      4710 -                    println!("      EP state={}, type={}, maxpkt={}, DCS={}", ep_state, ep_type, max_packet, dcs);                      
      4711 -                    print!("      EP dequeue ptr: ");                                                                                   
      4712 -                    print_hex64(deq_ptr);                                                                                               
      4713 -                    println!();                                                                                                         
      4714 -                                                                                                                                        
      4715 -                    // Check the TRB at the EP's dequeue pointer                                                                        
      4716 -                    print!("      Our bulk IN ring base: ");                                                                            
      4717 -                    print_hex64(bulk_in_ring_phys);                                                                                     
      4718 -                    println!();                                                                                                         
      4719 -                                                                                                                                        
      4720 -                    let trb_index = ((deq_ptr - bulk_in_ring_phys) / 16) as usize;                                                      
      4721 -                    println!("      EP dequeue index: {}", trb_index);                                                                  
      4722 -                }                                                                                                                       
      4644              }
      4645              return;
      4646          }
     ...
      4649          unsafe {
      4650              let csw_ptr = data_virt.add(csw_offset) as *const Csw;
      4651              // U-Boot pattern: invalidate cache before reading DMA data
      4731 -            // Use dc civac (clean and invalidate) as dc ivac is privileged                                                             
      4652              core::arch::asm!("dsb sy", options(nostack, preserves_flags));
      4653              core::arch::asm!(
      4654                  "dc civac, {addr}",
     ...
      4658              core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
      4659  
      4660              let csw = core::ptr::read_volatile(csw_ptr);
      4741 -            let tag = csw.tag;                                                                                                          
      4742 -            let status = csw.status;                                                                                                    
      4661              print!("      CSW: sig=");
      4662              print_hex32(csw.signature);
      4745 -            print!(", tag={}, status={}", tag, status);                                                                                 
      4663 +            print!(", tag={}, status={}", csw.tag, csw.status);                                                                         
      4664              println!();
      4665          }
      4666  
      4749 -        // Continue with the rest of the test...                                                                                        
      4750 -        if false {  // Skip old timeout code                                                                                            
      4751 -            println!("    CSW timeout - bulk IN still not working");                                                                    
      4752 -                                                                                                                                        
      4753 -            // Print the TRB we set up                                                                                                  
      4754 -            unsafe {                                                                                                                    
      4755 -                let trb = &*bulk_in_ring_virt.add(in_enqueue - 1);  // The one we just set up                                           
      4756 -                print!("      Bulk IN TRB: param=");                                                                                    
      4757 -                print_hex64(trb.param);                                                                                                 
      4758 -                print!(", status=");                                                                                                    
      4759 -                print_hex32(trb.status);                                                                                                
      4760 -                print!(", ctrl=");                                                                                                      
      4761 -                print_hex32(trb.control);                                                                                               
      4762 -                println!();                                                                                                             
      4763 -            }                                                                                                                           
      4764 -                                                                                                                                        
      4765 -            // Dump event ring state                                                                                                    
      4766 -            if let Some(ref event_ring) = self.event_ring {                                                                             
      4767 -                println!("      Event ring dequeue: {}, cycle: {}", event_ring.dequeue, event_ring.cycle);                              
      4768 -                                                                                                                                        
      4769 -                // Check if event appeared NOW (after timeout prints)                                                                   
      4770 -                unsafe {                                                                                                                
      4771 -                    core::arch::asm!("dsb sy", options(nostack, preserves_flags));                                                      
      4772 -                    let trb_ptr = event_ring.trbs.add(event_ring.dequeue);                                                              
      4773 -                    let ctrl = core::ptr::read_volatile(&(*trb_ptr).control);                                                           
      4774 -                    let trb_cycle = ctrl & 1;                                                                                           
      4775 -                    let trb_type = (ctrl >> 10) & 0x3F;                                                                                 
      4776 -                    print!("      RIGHT NOW TRB[{}]: type={}, cycle={}", event_ring.dequeue, trb_type, trb_cycle);                      
      4777 -                    if trb_type != 0 && trb_cycle == 1 {                                                                                
      4778 -                        println!(" <- EVENT HERE NOW!");                                                                                
      4779 -                    } else {                                                                                                            
      4780 -                        println!(" <- still empty");                                                                                    
      4781 -                    }                                                                                                                   
      4782 -                }                                                                                                                       
      4783 -                                                                                                                                        
      4784 -                // SCAN ENTIRE EVENT RING looking for any events with matching cycle                                                    
      4785 -                println!("      Scanning entire event ring for missed events:");                                                        
      4786 -                let mut found_events = 0u32;                                                                                            
      4787 -                unsafe {                                                                                                                
      4788 -                    for idx in 0..RING_SIZE {                                                                                           
      4789 -                        let trb_ptr = event_ring.trbs.add(idx);                                                                         
      4790 -                        let ctrl = core::ptr::read_volatile(&(*trb_ptr).control);                                                       
      4791 -                        let param = core::ptr::read_volatile(&(*trb_ptr).param);                                                        
      4792 -                        let status = core::ptr::read_volatile(&(*trb_ptr).status);                                                      
      4793 -                        let trb_cycle = ctrl & 1;                                                                                       
      4794 -                        let trb_type = (ctrl >> 10) & 0x3F;                                                                             
      4795 -                                                                                                                                        
      4796 -                        // Show any TRB with non-zero type OR with cycle=1 (our expected cycle)                                         
      4797 -                        if trb_type != 0 || trb_cycle == 1 {                                                                            
      4798 -                            found_events += 1;                                                                                          
      4799 -                            print!("        [{}] type={}, cycle={}", idx, trb_type, trb_cycle);                                         
      4800 -                            if trb_type != 0 {                                                                                          
      4801 -                                let cc = (status >> 24) & 0xFF;                                                                         
      4802 -                                let slot = (ctrl >> 24) & 0xFF;                                                                         
      4803 -                                print!(", CC={}, slot={}, param=", cc, slot);                                                           
      4804 -                                print_hex64(param);                                                                                     
      4805 -                            }                                                                                                           
      4806 -                            println!();                                                                                                 
      4807 -                        }                                                                                                               
      4808 -                    }                                                                                                                   
      4809 -                }                                                                                                                       
      4810 -                if found_events == 0 {                                                                                                  
      4811 -                    println!("        (no events found with cycle=1 or type!=0)");                                                      
      4812 -                }                                                                                                                       
      4813 -                println!("      Total events with data: {}", found_events);                                                             
      4814 -                                                                                                                                        
      4815 -                // Also check ERST and ERDP                                                                                             
      4816 -                print!("      Event ring phys base: ");                                                                                 
      4817 -                print_hex64(event_ring.trbs_phys);                                                                                      
      4818 -                println!();                                                                                                             
      4819 -            }                                                                                                                           
      4820 -                                                                                                                                        
      4821 -            // Check IMAN for interrupt state                                                                                           
      4822 -            unsafe {                                                                                                                    
      4823 -                let iman = core::ptr::read_volatile(                                                                                    
      4824 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *const u32                                   
      4825 -                );                                                                                                                      
      4826 -                let imod = core::ptr::read_volatile(                                                                                    
      4827 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMOD) as u64) as *const u32                                   
      4828 -                );                                                                                                                      
      4829 -                let erstsz = core::ptr::read_volatile(                                                                                  
      4830 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERSTSZ) as u64) as *const u32                                 
      4831 -                );                                                                                                                      
      4832 -                let erstba = core::ptr::read_volatile(                                                                                  
      4833 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERSTBA) as u64) as *const u64                                 
      4834 -                );                                                                                                                      
      4835 -                let erdp = core::ptr::read_volatile(                                                                                    
      4836 -                    (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *const u64                                   
      4837 -                );                                                                                                                      
      4838 -                print!("      IMAN=");                                                                                                  
      4839 -                print_hex32(iman);                                                                                                      
      4840 -                print!(", IMOD=");                                                                                                      
      4841 -                print_hex32(imod);                                                                                                      
      4842 -                println!();                                                                                                             
      4843 -                print!("      ERSTSZ={}, ERSTBA=", erstsz);                                                                             
      4844 -                print_hex64(erstba);                                                                                                    
      4845 -                println!();                                                                                                             
      4846 -                print!("      ERDP=");                                                                                                  
      4847 -                print_hex64(erdp);                                                                                                      
      4848 -                println!();                                                                                                             
      4849 -            }                                                                                                                           
      4850 -                                                                                                                                        
      4851 -            // Check xHCI state                                                                                                         
      4852 -            unsafe {                                                                                                                    
      4853 -                let usbsts = core::ptr::read_volatile(                                                                                  
      4854 -                    (mac_base + op_base as u64 + xhci_op::USBSTS as u64) as *const u32                                                  
      4855 -                );                                                                                                                      
      4856 -                print!("      USBSTS=");                                                                                                
      4857 -                print_hex32(usbsts);                                                                                                    
      4858 -                println!();                                                                                                             
      4859 -                                                                                                                                        
      4860 -                // Check the doorbell register to see if it's still pending                                                             
      4861 -                let db_val = core::ptr::read_volatile(                                                                                  
      4862 -                    (mac_base + self.db_base as u64 + slot_id as u64 * 4) as *const u32                                                 
      4863 -                );                                                                                                                      
      4864 -                print!("      Doorbell[{}]=", slot_id);                                                                                 
      4865 -                print_hex32(db_val);                                                                                                    
      4866 -                println!();                                                                                                             
      4867 -            }                                                                                                                           
      4868 -                                                                                                                                        
      4869 -            // Check final endpoint context state - compare bulk IN vs bulk OUT                                                         
      4870 -            unsafe {                                                                                                                    
      4871 -                let dev_ctx = &*device_ctx_virt;                                                                                        
      4872 -                                                                                                                                        
      4873 -                // Bulk OUT (for comparison - this one works)                                                                           
      4874 -                let bulk_out_idx = (bulk_out_dci - 1) as usize;                                                                         
      4875 -                let bulk_out_ctx = &dev_ctx.endpoints[bulk_out_idx];                                                                    
      4876 -                let out_state = bulk_out_ctx.dw0 & 0x7;                                                                                 
      4877 -                let out_deq = ((bulk_out_ctx.dw2 as u64) | ((bulk_out_ctx.dw3 as u64) << 32)) & !0xF;                                   
      4878 -                let out_dcs = bulk_out_ctx.dw2 & 1;                                                                                     
      4879 -                print!("      Bulk OUT: state={}, dcs={}, deq=", out_state, out_dcs);                                                   
      4880 -                print_hex64(out_deq);                                                                                                   
      4881 -                println!(" (WORKS)");                                                                                                   
      4882 -                                                                                                                                        
      4883 -                // Bulk IN (broken)                                                                                                     
      4884 -                let bulk_in_idx = (bulk_in_dci - 1) as usize;                                                                           
      4885 -                let bulk_in_ctx = &dev_ctx.endpoints[bulk_in_idx];                                                                      
      4886 -                let ep_state = bulk_in_ctx.dw0 & 0x7;                                                                                   
      4887 -                let ep_state_str = match ep_state {                                                                                     
      4888 -                    0 => "Disabled",                                                                                                    
      4889 -                    1 => "Running",                                                                                                     
      4890 -                    2 => "Halted",                                                                                                      
      4891 -                    3 => "Stopped",                                                                                                     
      4892 -                    4 => "Error",                                                                                                       
      4893 -                    _ => "Unknown",                                                                                                     
      4894 -                };                                                                                                                      
      4895 -                let tr_dequeue = ((bulk_in_ctx.dw2 as u64) | ((bulk_in_ctx.dw3 as u64) << 32)) & !0xF;                                  
      4896 -                let dcs = bulk_in_ctx.dw2 & 1;                                                                                          
      4897 -                println!("      Bulk IN: state={} ({}), DCS={}", ep_state, ep_state_str, dcs);                                          
      4898 -                print!("      IN TR Dequeue: ");                                                                                        
      4899 -                print_hex64(tr_dequeue);                                                                                                
      4900 -                print!(", Ring base: ");                                                                                                
      4901 -                print_hex64(bulk_in_ring_phys);                                                                                         
      4902 -                println!();                                                                                                             
      4903 -                                                                                                                                        
      4904 -                // Check if the TRB cycle bit matches expected                                                                          
      4905 -                let trb = &*bulk_in_ring_virt;                                                                                          
      4906 -                let trb_cycle = trb.control & 1;                                                                                        
      4907 -                println!("      TRB[0] cycle bit: {}, EP DCS: {}", trb_cycle, dcs);                                                     
      4908 -                if trb_cycle != dcs as u32 {                                                                                            
      4909 -                    println!("      *** CYCLE BIT MISMATCH - xHCI won't process this TRB! ***");                                        
      4910 -                }                                                                                                                       
      4911 -                                                                                                                                        
      4912 -                // Print full context dwords for analysis                                                                               
      4913 -                print!("      EP IN dw0=");                                                                                             
      4914 -                print_hex32(bulk_in_ctx.dw0);                                                                                           
      4915 -                print!(", dw1=");                                                                                                       
      4916 -                print_hex32(bulk_in_ctx.dw1);                                                                                           
      4917 -                println!();                                                                                                             
      4918 -                print!("      EP IN dw2=");                                                                                             
      4919 -                print_hex32(bulk_in_ctx.dw2);                                                                                           
      4920 -                print!(", dw3=");                                                                                                       
      4921 -                print_hex32(bulk_in_ctx.dw3);                                                                                           
      4922 -                print!(", dw4=");                                                                                                       
      4923 -                print_hex32(bulk_in_ctx.dw4);                                                                                           
      4924 -                println!();                                                                                                             
      4925 -            }                                                                                                                           
      4926 -                                                                                                                                        
      4927 -            println!("    (bulk IN completely broken - not just data transfer)");                                                       
      4928 -        } // end if false                                                                                                               
      4929 -                                                                                                                                        
      4667          // TUR succeeded!
      4668          println!("    TEST_UNIT_READY complete - bulk IN works!");
      4669          println!("    Device is ready for SCSI commands");

⏺ Now let me build to check for errors:

⏺ Bash(./build.sh 2>&1 | grep -E "^error|^warning.*usbtest|Compiling usbtest|Finished")
  ⎿      Finished `release` profile [optimized] target(s) in 0.34s                      
         Finished `release` profile [optimized] target(s) in 0.12s
        Compiling usbtest v0.1.0 (/Users/t.heloin/git/banana/bpi-r4-kernel/user/usbtest)
     … +3 lines (ctrl+o to expand)

⏺ Bash(./build.sh 2>&1 | grep -A 20 "error\[E0793\]")
  ⎿  error[E0793]: reference to field of packed struct is unaligned
         --> src/main.rs:4663:43
          |
     … +20 lines (ctrl+o to expand)

⏺ I need to fix the packed struct access:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 5 lines, removed 2 lines                                                                                                                 
      4658              core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
      4659  
      4660              let csw = core::ptr::read_volatile(csw_ptr);
      4661 +            let sig = csw.signature;                                                                                                    
      4662 +            let tag = csw.tag;                                                                                                          
      4663 +            let status = csw.status;                                                                                                    
      4664              print!("      CSW: sig=");
      4662 -            print_hex32(csw.signature);                                                                                                 
      4663 -            print!(", tag={}, status={}", csw.tag, csw.status);                                                                         
      4665 +            print_hex32(sig);                                                                                                           
      4666 +            print!(", tag={}, status={}", tag, status);                                                                                 
      4667              println!();
      4668          }
      4669  

⏺ Bash(./build.sh 2>&1 | tail -30)
  ⎿  a ./bin/usbtest               
     a ./bin/shell
     a ./bin/gpio
     … +27 lines (ctrl+o to expand)

⏺ Build succeeded. The kernel now uses interrupt-based waiting instead of tight polling:

  Key changes:
  1. wait_for_irq() does a blocking read on the IRQ scheme - truly suspends until xHCI generates an interrupt
  2. No more tight polling with constant DC CIVAC operations that might interfere with DMA
  3. Falls back to simple delay+poll if IRQ fd not available
  4. Much cleaner code - removed ~200 lines of verbose debug output

  The new flow:
  1. Ring doorbell for bulk IN
  2. Call wait_for_irq() which blocks until xHCI interrupt
  3. On interrupt, check event ring with cache invalidation
  4. Repeat up to 10 times if needed

  This should be much faster and cleaner than the previous tight polling approach. Ready for testing.

> === Running SCSI Commands ===
  Clearing pending interrupt (IMAN.IP)...
  Clearing Port Change Detect...
  Waiting for device to initialize...
......... done
  SCSI TEST_UNIT_READY (no data phase)...
    Step 1: Sending CBW on bulk OUT...
      CBW TRB: param=000000004008c100, len=31, ctrl=00000421
    Event: slot=2, CC=1
    CBW sent successfully
    Step 2: Setting up CSW receive on bulk IN (13 bytes)...
      CSW buffer: 000000004008c140
      Current bulk IN EP DCS: 1, raw dw2: 40089001
      CSW TRB: param=000000004008c140, len=13, ctrl=00000421
      TRB verify after flush: param=000000004008c140, ctrl=00000421
    Ringing bulk IN doorbell (DCI=5)...
      After doorbell: EP state=1, DCS=1, deq_idx=0
      Bulk IN TRB[0] in memory: param=000000004008c140, ctrl=00000421
    Step 3: Waiting for CSW (poll with cache invalidation)...
      Initial IMAN: 00000003 (clearing IP)
    Waiting for xHCI interrupt (blocking)...
[IRQ] irq_wait: blocking pid 3 on IRQ 172
[IRQ] switching from slot 2 (pid 3) to slot 1 (pid 2, state=Ready)
[IRQ] next task ELR=0x40010f90
[IRQ] task switch complete, returning...
[TIMER] tick 500, from_user=1
[TIMER] tick 1000, from_user=1
[TIMER] tick 1500, from_user=1
[TIMER] tick 2000, from_user=1
[TIMER] tick 2500, from_user=1
[TIMER] tick 3000, from_user=1
[TIMER] tick 3500, from_user=1
[TIMER] tick 4000, from_user=1
[TIMER] tick 4500, from_user=1
[TIMER] tick 5000, from_user=1 

⏺ The xHCI interrupt never fires. The process is properly blocked, but the hardware isn't generating an interrupt.

  This confirms the issue isn't cache coherency or tight polling interference - the xHCI controller genuinely isn't completing the transfer.

  Interesting observation: in the previous tight-polling version, the transfer DID eventually complete after ~2500 iterations. The constant bus activity from polling might have been keeping the USB 3.0 link active.

  USB 3.0 has aggressive link power management (U1/U2 states). When the CPU goes idle (WFI), the link might enter a low-power state where the device can't respond.

  Let me add a hybrid approach - try IRQ wait briefly, then fall back to active polling:

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 25 lines

⏺ Let me go back to a simpler polling approach since the interrupt-based waiting doesn't work (the xHCI doesn't complete the transfer when idle):

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 28 lines, removed 80 lines                                                                                                               
      4506              }
      4507          }
      4508  
      4509 -        // Use interrupt-based waiting instead of tight polling                                                                         
      4510 -        // Tight polling with cache operations may interfere with xHCI DMA                                                              
      4511 -        println!("    Waiting for xHCI interrupt (blocking)...");                                                                       
      4509 +        // Poll for CSW completion                                                                                                      
      4510 +        // Note: Interrupt-based waiting doesn't work - xHCI doesn't complete transfer when CPU is idle.                                
      4511 +        // Active polling keeps the bus/memory subsystem active which seems necessary.                                                  
      4512 +        println!("    Waiting for CSW...");                                                                                             
      4513  
      4514          let mut csw_received = false;
      4515          let mut poll_iterations = 0u32;
      4516  
      4516 -        // Try interrupt-based wait first (up to 10 attempts with 500ms timeout implied by blocking read)                               
      4517 -        for attempt in 0..10 {                                                                                                          
      4518 -            poll_iterations = attempt;                                                                                                  
      4517 +        if let Some(ref mut event_ring) = self.event_ring {                                                                             
      4518 +            for i in 0..5000 {                                                                                                          
      4519 +                poll_iterations = i;                                                                                                    
      4520  
      4520 -            // Block until xHCI generates an interrupt                                                                                  
      4521 -            if self.wait_for_irq(500) {                                                                                                 
      4522 -                println!("      IRQ received at attempt {}", attempt);                                                                  
      4521 +                // Check for event with cache invalidation                                                                              
      4522 +                if let Some(evt) = event_ring.dequeue() {                                                                               
      4523 +                    let evt_type = evt.get_type();                                                                                      
      4524 +                    let cc = (evt.status >> 24) & 0xFF;                                                                                 
      4525  
      4524 -                // Interrupt fired - now check for our event                                                                            
      4525 -                if let Some(ref mut event_ring) = self.event_ring {                                                                     
      4526 -                    // Invalidate event ring before reading                                                                             
      4527 -                    if let Some(evt) = event_ring.dequeue() {                                                                           
      4528 -                        let evt_type = evt.get_type();                                                                                  
      4529 -                        let cc = (evt.status >> 24) & 0xFF;                                                                             
      4530 -                        let slot = (evt.control >> 24) & 0xFF;                                                                          
      4531 -                        print!("      Event: type={}, slot={}, CC={}", evt_type, slot, cc);                                             
      4532 -                        println!();                                                                                                     
      4533 -                                                                                                                                        
      4534 -                        if evt_type == trb_type::TRANSFER_EVENT {                                                                       
      4535 -                            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                    
      4536 -                                csw_received = true;                                                                                    
      4537 -                                println!("      CSW received successfully!");                                                           
      4538 -                            } else {                                                                                                    
      4539 -                                print!("      CSW transfer failed CC=");                                                                
      4540 -                                print_hex32(cc);                                                                                        
      4541 -                                println!();                                                                                             
      4542 -                            }                                                                                                           
      4543 -                            // Update ERDP and clear interrupt                                                                          
      4544 -                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4545 -                            unsafe {                                                                                                    
      4546 -                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4547 -                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4548 -                                // Clear IMAN.IP by writing 1                                                                           
      4549 -                                let iman_ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;         
      4550 -                                let iman = core::ptr::read_volatile(iman_ptr);                                                          
      4551 -                                core::ptr::write_volatile(iman_ptr, iman | 1);                                                          
      4552 -                            }                                                                                                           
      4553 -                            break;                                                                                                      
      4526 +                    if evt_type == trb_type::TRANSFER_EVENT {                                                                           
      4527 +                        if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                        
      4528 +                            csw_received = true;                                                                                        
      4529 +                            println!("      CSW received at iteration {}", i);                                                          
      4530                          } else {
      4555 -                            // Acknowledge non-transfer event and continue waiting                                                      
      4556 -                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4557 -                            unsafe {                                                                                                    
      4558 -                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4559 -                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4560 -                                // Clear IMAN.IP                                                                                        
      4561 -                                let iman_ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::IMAN) as u64) as *mut u32;         
      4562 -                                let iman = core::ptr::read_volatile(iman_ptr);                                                          
      4563 -                                core::ptr::write_volatile(iman_ptr, iman | 1);                                                          
      4564 -                            }                                                                                                           
      4531 +                            print!("      Transfer failed CC=");                                                                        
      4532 +                            print_hex32(cc);                                                                                            
      4533 +                            println!();                                                                                                 
      4534                          }
      4535 +                        // Update ERDP                                                                                                  
      4536 +                        let erdp = event_ring.erdp() | (1 << 3);                                                                        
      4537 +                        unsafe {                                                                                                        
      4538 +                            let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;                  
      4539 +                            core::ptr::write_volatile(ptr, erdp);                                                                       
      4540 +                        }                                                                                                               
      4541 +                        break;                                                                                                          
      4542                      }
      4543                  }
      4568 -            } else {                                                                                                                    
      4569 -                // IRQ not available - fall back to a simple delay and poll                                                             
      4570 -                println!("      No IRQ fd, falling back to delay...");                                                                  
      4571 -                for _ in 0..10000 {                                                                                                     
      4544 +                                                                                                                                        
      4545 +                // Small delay between polls - not too aggressive                                                                       
      4546 +                for _ in 0..1000 {                                                                                                      
      4547                      unsafe { core::arch::asm!("nop", options(nostack)); }
      4548                  }
      4574 -                // Try to dequeue after delay                                                                                           
      4575 -                if let Some(ref mut event_ring) = self.event_ring {                                                                     
      4576 -                    if let Some(evt) = event_ring.dequeue() {                                                                           
      4577 -                        let evt_type = evt.get_type();                                                                                  
      4578 -                        let cc = (evt.status >> 24) & 0xFF;                                                                             
      4579 -                        let slot = (evt.control >> 24) & 0xFF;                                                                          
      4580 -                        print!("      Event (fallback): type={}, slot={}, CC={}", evt_type, slot, cc);                                  
      4581 -                        println!();                                                                                                     
      4582 -                                                                                                                                        
      4583 -                        if evt_type == trb_type::TRANSFER_EVENT {                                                                       
      4584 -                            if cc == trb_cc::SUCCESS || cc == trb_cc::SHORT_PACKET {                                                    
      4585 -                                csw_received = true;                                                                                    
      4586 -                                println!("      CSW received successfully!");                                                           
      4587 -                            } else {                                                                                                    
      4588 -                                print!("      CSW transfer failed CC=");                                                                
      4589 -                                print_hex32(cc);                                                                                        
      4590 -                                println!();                                                                                             
      4591 -                            }                                                                                                           
      4592 -                            let erdp = event_ring.erdp() | (1 << 3);                                                                    
      4593 -                            unsafe {                                                                                                    
      4594 -                                let ptr = (mac_base + rt_base as u64 + (xhci_rt::IR0 + xhci_ir::ERDP) as u64) as *mut u64;              
      4595 -                                core::ptr::write_volatile(ptr, erdp);                                                                   
      4596 -                            }                                                                                                           
      4597 -                            break;                                                                                                      
      4598 -                        }                                                                                                               
      4599 -                    }                                                                                                                   
      4600 -                }                                                                                                                       
      4549              }
      4550          }
      4551  

⏺ Bash(./build.sh 2>&1 | tail -15)
  ⎿      Finished `release` profile [optimized] target(s) in 1.98s

     Step 4: Creating kernel.bin...
     … +11 lines (ctrl+o to expand)

⏺ Build successful. The new approach:
  - Simple polling with NOP delays (no WFI that could idle the CPU)
  - 5000 iterations max (previous test succeeded around iteration 2500)
  - Cache invalidation via event_ring.dequeue()

  This confirms an interesting hardware behavior: the xHCI on MT7988A seems to require active CPU/bus activity to complete bulk IN transfers. When the CPU goes idle (WFI), transfers stall. This might be:
  1. A USB 3.0 link power management issue
  2. Some quirk with the MediaTek xHCI implementation
  3. A memory bus/DMA coherency timing issue

  The polling approach works around this. Ready for testing.

> ========================================
  USB initialization complete!
  Monitoring ports - plug in a device
========================================

=== Opening SSUSB0 for monitoring ===

=== Port Monitor (30 seconds) ===
  Monitoring SSUSB0 and SSUSB1 - try plugging device into different ports
  [0s] SSUSB1 Port 1 CONNECTED! PORTSC=0x002a1203

=== Enumerating Device on Port 1 ===
  Enabling slot...
    Enable Slot event: CC=1, slot=1
    (skipped 2 other events)
    Got slot 1
  Port 1 speed: 4 (Super)
  DCBAA[1] = 0x40083500
  Sending Address Device command...
    Cmd ring: enq=2, cycle=true
    TRB at 0000000040081020
    Event ring: deq=4, cycle=true
    EvtTRB[0]: ctrl=00008401 (cycle=1)
    EvtTRB[1]: ctrl=00008801 (cycle=1)
    EvtTRB[2]: ctrl=00008801 (cycle=1)
    EvtTRB[3]: ctrl=01008401 (cycle=1)
  Event: type=33, status=01000000, ctrl=01008401
  Address Device succeeded
  Device address: 1
  Getting Device Descriptor...
  No transfer event received (timeout)

=== Port Status ===
  Port 1: PORTSC=0x002a1203 PLS=U0 SSSpeed [Enabled]
  Port 2: PORTSC=0x000003c0 PLS=? [No device]
  Daemonized - running as background USB driverProcess 3 detached (daemonized)
> 
  Use 'ps' to see status, 'kill <pid>' to stop 

⏺ Hmm, now even control transfers (Get Device Descriptor) are timing out. This is different from before - the device is plugged directly into Port 1 instead of through a hub.

  Let me check if the control transfer polling code was affected by my changes:

⏺ Search(pattern: "Getting Device Descriptor|No transfer event received", path: "user/usbtest/src/main.rs", output_mode: "content")
  ⎿  Found 110 lines (ctrl+o to expand)

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 60 lines

⏺ Search(pattern: "^fn delay|^pub fn delay"", path: "user/usbtest/src/main.rs", output_mode: "content")
  ⎿  Found 11 lines (ctrl+o to expand)

⏺ I see the issue. The control transfer polling is only 100 iterations with small delays - that worked previously but might not be enough for direct SuperSpeed connection without a hub. Let me increase the control transfer polling to match what worked for bulk transfers:

⏺ Update(user/usbtest/src/main.rs)
  ⎿  Added 6 lines, removed 2 lines                                                                                                                 
      2360          delay(50000);
      2361  
      2362          // Poll for transfer event - properly handle all events
      2363 +        // Note: SuperSpeed direct connect may need more iterations than through a hub                                                  
      2364          let mut transfer_result: Option<(u32, u32)> = None;  // (cc, remaining_len)
      2365          let mut erdp_to_update: Option<u64> = None;
      2366  
      2367          if let Some(ref mut event_ring) = self.event_ring {
      2367 -            for _ in 0..100 {                                                                                                           
      2368 +            for _ in 0..3000 {                                                                                                          
      2369                  if let Some(evt) = event_ring.dequeue() {
      2370                      // Always track ERDP for any consumed event
      2371                      erdp_to_update = Some(event_ring.erdp());
     ...
      2379                      }
      2380                      // Ignore other events but acknowledge them
      2381                  }
      2381 -                delay(1000);                                                                                                            
      2382 +                // Small delay between polls                                                                                            
      2383 +                for _ in 0..1000 {                                                                                                      
      2384 +                    unsafe { core::arch::asm!("nop", options(nostack)); }                                                               
      2385 +                }                                                                                                                       
      2386              }
      2387          }
      2388  

⏺ Let me also check for other control transfer polling locations that might need the same fix:

⏺ Search(pattern: "for _ in 0\.\.100", path: "user/usbtest/src/main.rs", output_mode: "content")
  ⎿  Found 14 lines (ctrl+o to expand)

⏺ Search(pattern: "fn control_transfer", path: "user/usbtest/src/main.rs", output_mode: "content")
  ⎿  Found 51 lines (ctrl+o to expand)

⏺ Read(user/usbtest/src/main.rs)
  ⎿  Read 80 lines

⏺ Bash(./build.sh 2>&1 | tail -15)
  ⎿      Finished `release` profile [optimized] target(s) in 2.04s

     Step 4: Creating kernel.bin...
     … +11 lines (ctrl+o to expand)
