//! Example drivers for the analog-to-digital converter module

use tm4c129x_hal::sysctl::{control_power, Domain, PowerControl, PowerState, RunMode};
use tm4c129x_hal::tm4c129x::{ADC0, GPIO_PORTB_AHB, GPIO_PORTD_AHB, GPIO_PORTE_AHB, GPIO_PORTK};

use tm4c129x_hal::gpio::GpioExt;

/// Number of samples to take and average in hardware
/// per sample delivered to the FIFO
#[allow(missing_docs)]
#[repr(u8)]
pub enum OverSampleMultiplier {
    _1x,
    _2x,
    _4x,
    _8x,
    _16x,
    _32x,
    _64x,
}

/// Configuration and sampling of ADC peripheral
#[allow(non_snake_case)]
pub struct ADC {
    ADC0: ADC0,
}

impl ADC {
    /// Power-on and configure ADC peripheral
    #[allow(non_snake_case)]
    pub fn new(
        ADC0: ADC0,
        gpiob: GPIO_PORTB_AHB,
        gpioe: GPIO_PORTE_AHB,
        gpiod: GPIO_PORTD_AHB,
        gpiok: GPIO_PORTK,
        power_control: &PowerControl,
        oversample_multiplier: OverSampleMultiplier,
        use_external_vref: bool,
    ) -> Self {
        //
        // 1. Set pin modes for all 20 ADC inputs
        //
        let pins_gpioe = gpioe.split(&power_control);
        pins_gpioe.pe0.into_floating_input(); // AIN3
        pins_gpioe.pe1.into_floating_input(); // AIN2
        pins_gpioe.pe2.into_floating_input(); // AIN1
        pins_gpioe.pe3.into_floating_input(); // AIN0

        pins_gpioe.pe4.into_floating_input(); // AIN8
        pins_gpioe.pe5.into_floating_input(); // AIN9

        let mut pins_gpiod = gpiod.split(&power_control);
        pins_gpiod.pd0.into_floating_input(); // AIN15
        pins_gpiod.pd1.into_floating_input(); // AIN14
        pins_gpiod.pd2.into_floating_input(); // AIN13
        pins_gpiod.pd3.into_floating_input(); // AIN12
        pins_gpiod.pd4.into_floating_input(); // AIN7
        pins_gpiod.pd5.into_floating_input(); // AIN6
        pins_gpiod.pd6.into_floating_input(); // AIN5
        pins_gpiod
            .pd7
            .unlock(&mut pins_gpiod.control)
            .into_floating_input(); // AIN4. not sure why it's locked

        let pins_gpiob = gpiob.split(&power_control);
        pins_gpiob.pb4.into_floating_input(); // AIN10
        pins_gpiob.pb5.into_floating_input(); // AIN11

        let pins_gpiok = gpiok.split(&power_control);
        pins_gpiok.pk0.into_floating_input(); // AIN16
        pins_gpiok.pk1.into_floating_input(); // AIN17
        pins_gpiok.pk2.into_floating_input(); // AIN18
        pins_gpiok.pk3.into_floating_input(); // AIN19

        //
        // 2. Power on the ADC peripheral
        //
        control_power(&power_control, Domain::Adc0, RunMode::Run, PowerState::On);

        //
        // 3. Configure ADC (must configure before enabling)
        //
        // Leave vref source as internal (default)
        if use_external_vref {
            ADC0.ctl.write(|w| w.vref().clear_bit());
        } else {
            ADC0.ctl.write(|w| w.vref().set_bit());
        }

        // Set hardware oversample-and-average multiplier
        {
            use OverSampleMultiplier::*;
            match oversample_multiplier {
                _1x => ADC0.sac.write(|w| w.avg().off()),
                _2x => ADC0.sac.write(|w| w.avg()._2x()),
                _4x => ADC0.sac.write(|w| w.avg()._4x()),
                _8x => ADC0.sac.write(|w| w.avg()._8x()),
                _16x => ADC0.sac.write(|w| w.avg()._16x()),
                _32x => ADC0.sac.write(|w| w.avg()._32x()),
                _64x => ADC0.sac.write(|w| w.avg()._64x()),
            }
        }

        // Use main oscillator as ADC clock
        ADC0.cc.write(|w| w.cs().mosc());

        //
        // 4. Enable DMA then ADC
        //
        ADC0.actss.write(|w| {
            w.aden0()
                .set_bit()
                .aden1()
                .set_bit()
                .aden2()
                .set_bit()
                .aden3()
                .set_bit()
        }); // Must be enabled before adc or it will crash with no error
        ADC0.actss.write(|w| {
            w.asen0()
                .set_bit()
                .asen1()
                .set_bit()
                .asen2()
                .set_bit()
                .asen3()
                .set_bit()
        }); // Must be configured before enabling or it will crash with no error

        Self { ADC0: ADC0 }
    }

    /// Read all 20 ADC channels
    pub fn sample(&mut self) -> [u16; 20] {
        //    set bank mux (ADCSSEMUX0)
        //    set sample mux (ADCSSMUX0)
        //    start sample (ADCPSSI)
        //    start conversion (automatic??)
        //    wait for sample (not ADCSSFSTAT0.EMPTY)
        //    while not ADCSSFSTAT0.EMPTY
        //        read value (ADCSSFIFO0)
        //    clear fifo overflow status (ADCOSTAT) by writing 1

        // Set flag that ADC sample sequencers should wait for gsync flag
        let _ = &self.ADC0.pssi.write(|w| w.syncwait().set_bit()); // indicate we should wait for gsync to start samples

        {
            // Sample sequencer 0
            // Select first ADC bank (inputs 0..=15)
            let _ = &self.ADC0.ssemux0.write_with_zero(|w| w);

            // Select inputs 0-7
            let _ = &self.ADC0.ssmux0.write(|w| {
                w.mux0()
                    .bits(0)
                    .mux1()
                    .bits(1)
                    .mux2()
                    .bits(2)
                    .mux3()
                    .bits(3)
                    .mux4()
                    .bits(4)
                    .mux5()
                    .bits(5)
                    .mux6()
                    .bits(6)
                    .mux7()
                    .bits(7)
            });

            // Set 4th sample as end of sequence
            let _ = &self.ADC0.ssctl0.write(|w| w.end7().set_bit());

            // Start sample sequencer that will capture samples
            let _ = &self.ADC0.pssi.write(|w| w.ss0().set_bit()); // sequencer init
        }

        {
            // Sample sequencer 1
            // Select first ADC bank (inputs 0..=15)
            let _ = &self.ADC0.ssemux1.write_with_zero(|w| w);

            // Select inputs 8-11
            unsafe {
                let _ = &self.ADC0.ssmux1.write(|w| {
                    w.mux0()
                        .bits(8)
                        .mux1()
                        .bits(9)
                        .mux2()
                        .bits(10)
                        .mux3()
                        .bits(11)
                });
            }

            // Set 4th sample as end of sequence
            let _ = &self.ADC0.ssctl1.write(|w| w.end3().set_bit());

            // Start sample sequencer that will capture samples
            let _ = &self.ADC0.pssi.write(|w| w.ss1().set_bit()); // sequencer init
        }

        {
            // Sample sequencer 2
            // Select first ADC bank (inputs 0..=15)
            let _ = &self.ADC0.ssemux2.write_with_zero(|w| w);

            // Select inputs 12-15
            unsafe {
                let _ = &self.ADC0.ssmux2.write(|w| {
                    w.mux0()
                        .bits(12)
                        .mux1()
                        .bits(13)
                        .mux2()
                        .bits(14)
                        .mux3()
                        .bits(15)
                });
            }

            // Set 4th sample as end of sequence
            let _ = &self.ADC0.ssctl2.write(|w| w.end3().set_bit());

            // Start sample sequencer that will capture samples
            let _ = &self.ADC0.pssi.write(|w| w.ss2().set_bit()); // sequencer init
        }

        // Set global sync to start sample all sequences
        let _ = &self.ADC0.pssi.write(|w| w.gsync().set_bit()); // Global sync

        // Wait until all 3 fifos are full
        while !self.ADC0.ssfstat0.read().full().bit_is_set()
            | !self.ADC0.ssfstat1.read().full().bit_is_set()
            | !self.ADC0.ssfstat2.read().full().bit_is_set()
        {
            // do nothing
        }

        // Read all 16 values we've sampled so far
        let mut adcvals = [0_u16; 20]; // there are 20 total, but we're only reading a few for now
        for i in 0..8_usize {
            adcvals[i] = *(&self.ADC0.ssfifo0.read().data().bits());
        }
        for i in 8..12_usize {
            adcvals[i] = *(&self.ADC0.ssfifo1.read().data().bits());
        }
        for i in 12..16_usize {
            adcvals[i] = *(&self.ADC0.ssfifo2.read().data().bits());
        }

        {
            // Sample sequencer 1
            // Select second ADC bank (inputs 16..20)
            let _ = &self.ADC0.ssemux1.write(|w| {
                w.emux0()
                    .set_bit()
                    .emux1()
                    .set_bit()
                    .emux2()
                    .set_bit()
                    .emux3()
                    .set_bit()
            });

            // Select inputs 16-19
            unsafe {
                let _ = &self.ADC0.ssmux1.write(|w| {
                    w.mux0()
                        .bits(16)
                        .mux1()
                        .bits(17)
                        .mux2()
                        .bits(18)
                        .mux3()
                        .bits(19)
                });
            }

            // Set 4th sample as end of sequence
            let _ = &self.ADC0.ssctl1.write(|w| w.end3().set_bit());

            // Start sample sequencer that will capture samples
            let _ = &self.ADC0.pssi.write(|w| w.ss1().set_bit()); // sequencer init
        }

        // Clear flag for syncing sequencers
        let _ = &self.ADC0.pssi.write(|w| w.syncwait().clear_bit()); // indicate we should wait for gsync to start samples

        // Raise sync flag to make sure sequencer starts sampling
        let _ = &self.ADC0.pssi.write(|w| w.gsync().set_bit()); // Global sync

        // Wait until SS1 fifo is full
        while !self.ADC0.ssfstat1.read().full().bit_is_set() {
            // do nothing
        }

        // Pull the last 4 samples from SS1
        for i in 16..20_usize {
            adcvals[i] = *(&self.ADC0.ssfifo1.read().data().bits());
        }

        adcvals
    }
}
