//! Handles board-specific CPU startup

use cortex_m;
use cortex_m_rt::{entry, exception, ExceptionFrame};

use super::board::{clocks, safe, Board};
use tm4c129x_hal::{gpio::GpioExt, serial, sysctl::SysctlExt, time::Bps};

use irq::{handler, scope, scoped_interrupts};

#[cfg(debug_assertions)]
use core::fmt::Write;

// This function must be implemented by the application that uses the crate
// and is the entry-point for that application after board initialization
extern "Rust" {
    fn stellaris_main(board: Board);
}

/// Performs what you might otherwise call 'C Startup'.
/// This routine is specified at the reset vector in the ISR vector table.
///
/// Copies global .data init from flash to SRAM and then
/// zeros the bss segment.
#[entry]
unsafe fn call_main() -> ! {
    // Initialize runtime-defined exception handlers before running any
    // application code or doing anything that might trigger them
    handler!(systick_default_handler = || {});
    handler!(pendsv_default_handler = || {});
    handler!(svcall_default_handler = || {});

    scope(|default| {
        default.register(Interrupt::SysTick, systick_default_handler);
        default.register(Interrupt::PendSV, pendsv_default_handler);
        default.register(Interrupt::SVCall, svcall_default_handler);

        let board = Board::new();
        stellaris_main(board);
    });

    loop {
        cortex_m::asm::wfi();
    }
}

/// A HardFault is an exception that occurs because of an error during
/// exception processing, or because an exception cannot be managed by any
/// other exception mechanism. HardFaults have a fixed priority of -1, meaning
/// they have higher priority than any exception with configurable priority.
#[exception]
unsafe fn HardFault(_sf: &ExceptionFrame) -> ! {
    // Need ITM support for this to work
    // iprintln!("EXCEPTION {:?} @ PC=0x{:08x}", Exception::active(), sf.pc);

    let peripherals = tm4c129x_hal::Peripherals::steal();
    let sysctl = peripherals.SYSCTL.constrain();
    let mut pins = peripherals.GPIO_PORTA_AHB.split(&sysctl.power_control);
    let mut uart = serial::Serial::uart0(
        peripherals.UART0,
        pins.pa1.into_af_push_pull(&mut pins.control),
        pins.pa0.into_af_push_pull(&mut pins.control),
        (),
        (),
        Bps(115200),
        serial::NewlineMode::SwapLFtoCRLF,
        clocks(),
        &sysctl.power_control,
    );

    // Debug formatter can panic, so this can't be run with panic_never
    #[cfg(debug_assertions)]
    writeln!(uart, "SF: {:?}", _sf).unwrap_or_default();

    safe();
}

/// A Non Maskable Interrupt (NMI) can be signalled by a peripheral or
/// triggered by software. This is the highest priority exception other than
/// reset. It is permanently enabled and has a fixed priority of -2. NMIs
/// cannot be:
/// * masked or prevented from activation by any other exception
/// * preempted by any exception other than Reset.
#[exception]
unsafe fn NonMaskableInt() {
    safe();
}

/// A MemManage fault is an exception that occurs because of a memory
/// protection related fault. The the fixed memory protection constraints
/// determines this fault, for both instruction and data memory transactions.
/// This fault is always used to abort instruction accesses to Execute Never
/// (XN) memory regions.
#[exception]
fn MemoryManagement() {
    safe();
}

/// A BusFault is an exception that occurs because of a memory related fault
/// for an instruction or data memory transaction. This might be from an error
/// detected on a bus in the memory system.
#[exception]
fn BusFault() {
    safe();
}

/// A UsageFault is an exception that occurs because of a fault related to instruction execution. This includes:
/// * an undefined instruction
/// * an illegal unaligned access
/// * invalid state on instruction execution
/// * an error on exception return.
/// The following can cause a UsageFault when the core is configured to report them:
/// * an unaligned address on word and halfword memory access
/// * division by zero.
#[exception]
fn UsageFault() {
    safe();
}

/// Debug monitor interrupt handler.
#[exception]
fn DebugMonitor() {
    // Nothing
}

/// A place-holder ISR used when we have nothing better to use.
#[exception]
unsafe fn DefaultHandler(_irq_number: i16) -> () {
    // Nothing
}

scoped_interrupts! {
    /// Exception interrupts that can be overridden by the user at runtime
    ///
    /// A SysTick exception is an exception the system timer generates when it
    /// reaches zero. Software can also generate a SysTick exception. In an OS
    /// environment, the processor can use this exception as system tick.
    ///
    /// A supervisor call (SVC) is an exception that is triggered by the SVC
    /// instruction. In an OS environment, applications can use SVC instructions
    /// to access OS kernel functions and device drivers.
    ///
    /// PendSV is an interrupt-driven request for system-level service. In an OS
    /// environment, use PendSV for context switching when no other exception is
    /// active.
    ///
    /// We're using the exception interface instead of the interrupt interface here
    /// because the tm4c hal doesn't have the cortex-m-rt interrupt definitions,
    /// which are normally autogenerated by svd2rust
    #[allow(missing_docs)]
    pub enum Interrupt {
        SysTick,
        SVCall,
        PendSV
    }

    use #[exception];
}
