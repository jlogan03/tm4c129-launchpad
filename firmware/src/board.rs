//! Hardware definitions capturing the configuration of the board
use embedded_hal::digital::v2::OutputPin;
use tm4c129x_hal::gpio::{gpiof::*, gpioj::*, gpion::*, GpioExt, Input, Output, PullUp, PushPull};
use tm4c129x_hal::sysctl::{
    control_power, reset, Clocks, CrystalFrequency, Domain, Oscillator, PllOutputFrequency,
    PowerControl, PowerState, RunMode, SysctlExt, SystemClock,
};
use tm4c129x_hal::time::Hertz;

use crate::drivers;
use crate::drivers::ethernet::{EmacR, EphyR, EthernetDriver};

#[derive(PartialEq, Clone, Copy)]
/// The Launchpad has two buttons
pub enum Button {
    /// SW1
    One,
    /// SW2
    Two,
}

/// Hardware definitions for the TM4C129-XL Launchpad board
#[allow(non_snake_case)]
pub struct Board {
    /// The core peripherals on the TM4C129x
    pub core_peripherals: tm4c129x_hal::CorePeripherals,
    /// Power gating for peripherals in the TM4C129x
    pub power_control: tm4c129x_hal::sysctl::PowerControl,

    /// LED D1
    pub led0: PN1<Output<PushPull>>,
    /// LED D2
    pub led1: PN0<Output<PushPull>>,
    /// LED D3
    pub led2: PF4<Output<PushPull>>,
    /// LED D4
    pub led3: PF0<Output<PushPull>>,

    /// Button SW1
    pub button0: PJ0<Input<PullUp>>,
    /// Button SW2
    pub button1: PJ1<Input<PullUp>>,

    /// GPIO control for GPIO port F
    pub portf_control: tm4c129x_hal::gpio::gpiof::GpioControl,
    /// GPIO control for GPIO port N
    pub portn_control: tm4c129x_hal::gpio::gpion::GpioControl,
    /// GPIO control for GPIO port J
    pub portj_control: tm4c129x_hal::gpio::gpioj::GpioControl,

    /// EMAC driver
    pub enet: EthernetDriver,

    #[doc = "WATCHDOG0"]
    pub WATCHDOG0: tm4c129x_hal::tm4c129x::WATCHDOG0,
    #[doc = "WATCHDOG1"]
    pub WATCHDOG1: tm4c129x_hal::tm4c129x::WATCHDOG1,

    #[doc = "GPIO_PORTA_AHB"]
    pub GPIO_PORTA_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTA_AHB,
    #[doc = "GPIO_PORTB_AHB"]
    pub GPIO_PORTB_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTB_AHB,
    #[doc = "GPIO_PORTC_AHB"]
    pub GPIO_PORTC_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTC_AHB,
    #[doc = "GPIO_PORTD_AHB"]
    pub GPIO_PORTD_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTD_AHB,
    #[doc = "GPIO_PORTE_AHB"]
    pub GPIO_PORTE_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTE_AHB,
    // #[doc = "GPIO_PORTF_AHB"]
    // pub GPIO_PORTF_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTF_AHB,
    #[doc = "GPIO_PORTG_AHB"]
    pub GPIO_PORTG_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTG_AHB,
    #[doc = "GPIO_PORTH_AHB"]
    pub GPIO_PORTH_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTH_AHB,
    // #[doc = "GPIO_PORTJ_AHB"]
    // pub GPIO_PORTJ_AHB: tm4c129x_hal::tm4c129x::GPIO_PORTJ_AHB,
    #[doc = "GPIO_PORTK"]
    pub GPIO_PORTK: tm4c129x_hal::tm4c129x::GPIO_PORTK,
    #[doc = "GPIO_PORTL"]
    pub GPIO_PORTL: tm4c129x_hal::tm4c129x::GPIO_PORTL,
    #[doc = "GPIO_PORTM"]
    pub GPIO_PORTM: tm4c129x_hal::tm4c129x::GPIO_PORTM,
    // #[doc = "GPIO_PORTN"]
    // pub GPIO_PORTN: tm4c129x_hal::tm4c129x::GPIO_PORTN,
    #[doc = "GPIO_PORTP"]
    pub GPIO_PORTP: tm4c129x_hal::tm4c129x::GPIO_PORTP,
    #[doc = "GPIO_PORTQ"]
    pub GPIO_PORTQ: tm4c129x_hal::tm4c129x::GPIO_PORTQ,

    #[doc = "SSI0"]
    pub SSI0: tm4c129x_hal::tm4c129x::SSI0,
    #[doc = "SSI1"]
    pub SSI1: tm4c129x_hal::tm4c129x::SSI1,
    #[doc = "SSI2"]
    pub SSI2: tm4c129x_hal::tm4c129x::SSI2,
    #[doc = "SSI3"]
    pub SSI3: tm4c129x_hal::tm4c129x::SSI3,

    #[doc = "UART0"]
    pub UART0: tm4c129x_hal::tm4c129x::UART0,
    #[doc = "UART1"]
    pub UART1: tm4c129x_hal::tm4c129x::UART1,
    #[doc = "UART2"]
    pub UART2: tm4c129x_hal::tm4c129x::UART2,
    #[doc = "UART3"]
    pub UART3: tm4c129x_hal::tm4c129x::UART3,
    #[doc = "UART4"]
    pub UART4: tm4c129x_hal::tm4c129x::UART4,
    #[doc = "UART5"]
    pub UART5: tm4c129x_hal::tm4c129x::UART5,
    #[doc = "UART6"]
    pub UART6: tm4c129x_hal::tm4c129x::UART6,
    #[doc = "UART7"]
    pub UART7: tm4c129x_hal::tm4c129x::UART7,

    #[doc = "I2C0"]
    pub I2C0: tm4c129x_hal::tm4c129x::I2C0,
    #[doc = "I2C1"]
    pub I2C1: tm4c129x_hal::tm4c129x::I2C1,
    #[doc = "I2C2"]
    pub I2C2: tm4c129x_hal::tm4c129x::I2C2,
    #[doc = "I2C3"]
    pub I2C3: tm4c129x_hal::tm4c129x::I2C3,
    #[doc = "I2C4"]
    pub I2C4: tm4c129x_hal::tm4c129x::I2C4,
    #[doc = "I2C5"]
    pub I2C5: tm4c129x_hal::tm4c129x::I2C5,
    #[doc = "I2C6"]
    pub I2C6: tm4c129x_hal::tm4c129x::I2C6,
    #[doc = "I2C7"]
    pub I2C7: tm4c129x_hal::tm4c129x::I2C7,
    #[doc = "I2C8"]
    pub I2C8: tm4c129x_hal::tm4c129x::I2C8,
    #[doc = "I2C9"]
    pub I2C9: tm4c129x_hal::tm4c129x::I2C9,

    #[doc = "PWM0"]
    pub PWM0: tm4c129x_hal::tm4c129x::PWM0,
    #[doc = "QEI0"]
    pub QEI0: tm4c129x_hal::tm4c129x::QEI0,
    #[doc = "TIMER0"]
    pub TIMER0: tm4c129x_hal::tm4c129x::TIMER0,
    #[doc = "TIMER1"]
    pub TIMER1: tm4c129x_hal::tm4c129x::TIMER1,
    #[doc = "TIMER2"]
    pub TIMER2: tm4c129x_hal::tm4c129x::TIMER2,
    #[doc = "TIMER3"]
    pub TIMER3: tm4c129x_hal::tm4c129x::TIMER3,
    #[doc = "TIMER4"]
    pub TIMER4: tm4c129x_hal::tm4c129x::TIMER4,
    #[doc = "TIMER5"]
    pub TIMER5: tm4c129x_hal::tm4c129x::TIMER5,
    #[doc = "TIMER6"]
    pub TIMER6: tm4c129x_hal::tm4c129x::TIMER6,
    #[doc = "TIMER7"]
    pub TIMER7: tm4c129x_hal::tm4c129x::TIMER7,

    #[doc = "ADC0"]
    pub ADC0: tm4c129x_hal::tm4c129x::ADC0,
    #[doc = "ADC1"]
    pub ADC1: tm4c129x_hal::tm4c129x::ADC1,

    #[doc = "COMP"]
    pub COMP: tm4c129x_hal::tm4c129x::COMP,

    #[doc = "CAN0"]
    pub CAN0: tm4c129x_hal::tm4c129x::CAN0,
    #[doc = "CAN1"]
    pub CAN1: tm4c129x_hal::tm4c129x::CAN1,

    #[doc = "USB0"]
    pub USB0: tm4c129x_hal::tm4c129x::USB0,
    #[doc = "EEPROM"]
    pub EEPROM: tm4c129x_hal::tm4c129x::EEPROM,
    #[doc = "SYSEXC"]
    pub SYSEXC: tm4c129x_hal::tm4c129x::SYSEXC,
    #[doc = "HIB"]
    pub HIB: tm4c129x_hal::tm4c129x::HIB,
    #[doc = "FLASH_CTRL"]
    pub FLASH_CTRL: tm4c129x_hal::tm4c129x::FLASH_CTRL,
    #[doc = "UDMA"]
    pub UDMA: tm4c129x_hal::tm4c129x::UDMA,
    #[doc = "EPI0"]
    pub EPI0: tm4c129x_hal::tm4c129x::EPI0,
    // #[doc = "EMAC0"]
    // pub EMAC0: tm4c129x_hal::tm4c129x::EMAC0,  // Consumed by EMACDriver
}

/// Clock speed defaults
static mut CLOCKS: Clocks = Clocks {
    osc: Hertz(25_000_000),
    sysclk: Hertz(120_000_000),
};

/// Get the current clock rate of the CPU
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

impl Board {
    // Initialize peripherals
    pub(crate) fn new() -> Board {
        let core_peripherals = match tm4c129x_hal::CorePeripherals::take() {
            Some(x) => x,
            None => loop {}, // This error occurs before the panic handler could even work
        };
        let peripherals = match tm4c129x_hal::Peripherals::take() {
            Some(x) => x,
            None => loop {}, // This error occurs before the panic handler could even work
        };

        let mut sysctl = peripherals.SYSCTL.constrain();

        // FPU
        unsafe {
            core_peripherals.SCB.cpacr.modify(|d| {
                d | (0x3 /* full */ << 20/* CP10 privilege */)
                    | (0x3 /* full */ << 22/* CP11 privilege */)
            });
        }

        // Clocks
        let system_clk_freq = PllOutputFrequency::_120mhz;
        sysctl.clock_setup.oscillator = Oscillator::Main(
            CrystalFrequency::_25mhz,
            SystemClock::UsePll(system_clk_freq),
        );
        unsafe {
            CLOCKS = sysctl.clock_setup.freeze();
        }

        // GPIO (LEDs and buttons)
        let pins_gpion = peripherals.GPIO_PORTN.split(&sysctl.power_control);
        let led1: PN0<Output<PushPull>> = pins_gpion.pn0.into_push_pull_output();
        let led0: PN1<Output<PushPull>> = pins_gpion.pn1.into_push_pull_output();

        let pins_gpiof = peripherals.GPIO_PORTF_AHB.split(&sysctl.power_control);
        let led3: PF0<Output<PushPull>> = pins_gpiof.pf0.into_push_pull_output();
        let led2: PF4<Output<PushPull>> = pins_gpiof.pf4.into_push_pull_output();

        let pins_gpioj = peripherals.GPIO_PORTJ_AHB.split(&sysctl.power_control);
        let button0 = pins_gpioj.pj0.into_pull_up_input();
        let button1 = pins_gpioj.pj1.into_pull_up_input();

        // Ethernet
        // Note the portions that use the power_control lock introduce a panic branch if they are run from
        // another module, so they must be run directly here or passed as a closure until the compiler/linker
        // get better at eliminating unreachable panic branches.
        //     Power-on and enable EMAC0 and EPHY0 peripherals
        emac_enable(&sysctl.power_control);
        //     Get MAC address from read-only memory
        let src_macaddr = drivers::ethernet::get_rom_macaddr(&peripherals.FLASH_CTRL);
        //     Initialize EMAC driver
        let enet: EthernetDriver = EthernetDriver::new(
            &sysctl.power_control,
            |pc| ephy_reset_power(pc),
            peripherals.EMAC0,
            system_clk_freq,
            src_macaddr,
            true,
            drivers::ethernet::PreambleLength::_7,
            drivers::ethernet::InterFrameGap::_96,
            drivers::ethernet::BackOffLimit::_1024,
            true,
            true,
            drivers::ethernet::TXThresholdDMA::_64,
            drivers::ethernet::RXThresholdDMA::_64,
            drivers::ethernet::BurstSizeDMA::_4,
            drivers::ethernet::BurstSizeDMA::_4,
        );

        Board {
            core_peripherals,
            power_control: sysctl.power_control,

            // --------- Board-specific ---------
            led0,
            led1,
            led2,
            led3,

            button0,
            button1,

            portf_control: pins_gpiof.control,
            portn_control: pins_gpion.control,
            portj_control: pins_gpioj.control,

            enet,

            // ----------------------------------
            WATCHDOG0: peripherals.WATCHDOG0,
            WATCHDOG1: peripherals.WATCHDOG1,

            GPIO_PORTA_AHB: peripherals.GPIO_PORTA_AHB,
            GPIO_PORTB_AHB: peripherals.GPIO_PORTB_AHB,
            GPIO_PORTC_AHB: peripherals.GPIO_PORTC_AHB,
            GPIO_PORTD_AHB: peripherals.GPIO_PORTD_AHB,
            GPIO_PORTE_AHB: peripherals.GPIO_PORTE_AHB,
            // GPIO_PORTF_AHB: peripherals.GPIO_PORTF_AHB,  // Consumed (board-specific)
            GPIO_PORTG_AHB: peripherals.GPIO_PORTG_AHB,
            GPIO_PORTH_AHB: peripherals.GPIO_PORTH_AHB,
            // GPIO_PORTJ_AHB: peripherals.GPIO_PORTJ_AHB,  // Consumed (board-specific)
            GPIO_PORTK: peripherals.GPIO_PORTK,
            GPIO_PORTL: peripherals.GPIO_PORTL,
            GPIO_PORTM: peripherals.GPIO_PORTM,
            // GPIO_PORTN: peripherals.GPIO_PORTN,  // Consumed (board-specific)
            GPIO_PORTP: peripherals.GPIO_PORTP,
            GPIO_PORTQ: peripherals.GPIO_PORTQ,

            SSI0: peripherals.SSI0,
            SSI1: peripherals.SSI1,
            SSI2: peripherals.SSI2,
            SSI3: peripherals.SSI3,

            UART0: peripherals.UART0,
            UART1: peripherals.UART1,
            UART2: peripherals.UART2,
            UART3: peripherals.UART3,
            UART4: peripherals.UART4,
            UART5: peripherals.UART5,
            UART6: peripherals.UART6,
            UART7: peripherals.UART7,

            I2C0: peripherals.I2C0,
            I2C1: peripherals.I2C1,
            I2C2: peripherals.I2C2,
            I2C3: peripherals.I2C3,
            I2C4: peripherals.I2C4,
            I2C5: peripherals.I2C5,
            I2C6: peripherals.I2C6,
            I2C7: peripherals.I2C7,
            I2C8: peripherals.I2C8,
            I2C9: peripherals.I2C9,

            PWM0: peripherals.PWM0,
            QEI0: peripherals.QEI0,
            TIMER0: peripherals.TIMER0,
            TIMER1: peripherals.TIMER1,
            TIMER2: peripherals.TIMER2,
            TIMER3: peripherals.TIMER3,
            TIMER4: peripherals.TIMER4,
            TIMER5: peripherals.TIMER5,
            TIMER6: peripherals.TIMER6,
            TIMER7: peripherals.TIMER7,

            ADC0: peripherals.ADC0,
            ADC1: peripherals.ADC1,

            COMP: peripherals.COMP,

            CAN0: peripherals.CAN0,
            CAN1: peripherals.CAN1,

            USB0: peripherals.USB0,
            EEPROM: peripherals.EEPROM,
            SYSEXC: peripherals.SYSEXC,
            HIB: peripherals.HIB,
            FLASH_CTRL: peripherals.FLASH_CTRL,
            UDMA: peripherals.UDMA,
            EPI0: peripherals.EPI0,
            // EMAC0: peripherals.EMAC0,
        }
    }
}

/// Unrecoverable error; cycle LEDs until reset
pub fn safe() -> ! {
    use embedded_hal::blocking::delay::DelayMs;
    let core_peripherals = unsafe { tm4c129x_hal::CorePeripherals::steal() };
    let p = unsafe { tm4c129x_hal::Peripherals::steal() };
    let pins = p.GPIO_PORTF_AHB.split(&p.SYSCTL.constrain().power_control);

    let mut delay = tm4c129x_hal::delay::Delay::new(core_peripherals.SYST, unsafe { &CLOCKS });
    let mut led0 = pins.pf1.into_push_pull_output();
    loop {
        let _ = led0.set_high().unwrap_or_default();
        delay.delay_ms(200u32);
        let _ = led0.set_low().unwrap_or_default();
        delay.delay_ms(200u32);
    }
}

/// Reset EMAC, then wait until it shows ready status
fn emac_reset_power(power_control: &PowerControl) -> EmacR {
    //   Get a handle to sysctl to check ready status
    let p = unsafe { &*tm4c129x_hal::tm4c129x::SYSCTL::ptr() };
    // Reset EMAC, then wait until SYSCTL sets ready status
    reset(power_control, Domain::Emac0);
    loop {
        if p.premac.read().r0().bit_is_set() {
            let emacr: EmacR = EmacR {};
            return emacr;
        }
    }
}

/// Reset EPHY, then wait until it shows ready status
fn ephy_reset_power(power_control: &PowerControl) -> EphyR {
    //   Get a handle to sysctl to check ready status
    let p = unsafe { &*tm4c129x_hal::tm4c129x::SYSCTL::ptr() };
    //   Reset EPHY, then wait until SYSCTL sets ready status
    reset(power_control, Domain::Ephy0);
    loop {
        if p.prephy.read().r0().bit_is_set() {
            let ephyr: EphyR = EphyR {};
            return ephyr;
        }
    }
}

/// Power-on and reset EMAC then EPHY
pub fn emac_enable(power_control: &PowerControl) {
    control_power(power_control, Domain::Emac0, RunMode::Run, PowerState::On);
    emac_reset_power(power_control);
    control_power(power_control, Domain::Ephy0, RunMode::Run, PowerState::On);
    ephy_reset_power(power_control);
}
