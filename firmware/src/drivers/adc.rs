//! Example drivers for the analog-to-digital converter module

use tm4c129x_hal::sysctl::{control_power, PowerControl};
use tm4c129x_hal::tm4c129x::{ADC0, GPIO_PORTB_AHB, GPIO_PORTD_AHB, GPIO_PORTE_AHB, GPIO_PORTK};

struct ADC {
    registers: ADC0,
    _gpio_portb: GPIO_PORTB_AHB,
    _gpio_portd: GPIO_PORTD_AHB,
    _gpio_porte: GPIO_PORTE_AHB,
    _gpio_portk: GPIO_PORTK,
}

impl ADC {
    fn new(
        adc_registers: ADC0,
        gpiob: GPIO_PORTB_AHB,
        gpioe: GPIO_PORTE_AHB,
        gpiod: GPIO_PORTD_AHB,
        gpiok: GPIO_PORTK,
        power_control: &PowerControl,
        oversample_multiplier: OverSampleMultiplier,
        sample_hold_multiplier: SampleHoldMultiplier,
        use_external_vref: bool
    ) -> Self {
        //
        // 1. Set pin modes for all 20 ADC inputs
        //
        let mut pins_gpioe = gpioe.split(&power_control);
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

        let mut pins_gpiob = gpiob.split(&power_control);
        pins_gpiob.pb4.into_floating_input(); // AIN10
        pins_gpiob.pb5.into_floating_input(); // AIN11

        let mut pins_gpiok = gpiok.split(&power_control);
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
            adc_registers.ctl.write(|w| w.vref().clear_bit());
        }
        else {
            adc_registers.ctl.write(|w| w.vref().set_bit());
        }
       
        // Set hardware oversample-and-average multiplier
        {
            use OverSampleMultiplier::*;
            match oversample_multiplier {
                _1x => adc_registers.sac.write(|w| w.avg().off()),
                _2x => adc_registers.sac.write(|w| w.avg()._2x()),
                _4x => adc_registers.sac.write(|w| w.avg()._4x()),
                _8x => adc_registers.sac.write(|w| w.avg()._8x()),
                _16x => adc_registers.sac.write(|w| w.avg()._16x()),
                _32x => adc_registers.sac.write(|w| w.avg()._32x()),
                _64x => adc_registers.sac.write(|w| w.avg()._64x()),
            }
        }
        // Set sample hold time multiplier
        {
        use SampleHoldMultiplier::*;
            match sample_hold_multiplier {
                _4x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._4().tsh1()._4().tsh2()._4().tsh3()._4().tsh4()._4().tsh5()._4().tsh0()._4().tsh7()._4());
                    adc_registers.sstsh1.write(|w| w.tsh0()._4().tsh1()._4().tsh2()._4().tsh3()._4());
                    adc_registers.sstsh2.write(|w| w.tsh0()._4().tsh1()._4().tsh2()._4().tsh3()._4());
                    adc_registers.sstsh3.write(|w| w.tsh0()._4());
                },
                _8x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._8().tsh1()._8().tsh2()._8().tsh3()._8().tsh4()._8().tsh5()._8().tsh0()._8().tsh7()._8());
                    adc_registers.sstsh1.write(|w| w.tsh0()._8().tsh1()._8().tsh2()._8().tsh3()._8());
                    adc_registers.sstsh2.write(|w| w.tsh0()._8().tsh1()._8().tsh2()._8().tsh3()._8());
                    adc_registers.sstsh3.write(|w| w.tsh0()._8());
                },
                _16x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._16().tsh1()._16().tsh2()._16().tsh3()._16().tsh4()._16().tsh5()._16().tsh0()._16().tsh7()._16());
                    adc_registers.sstsh1.write(|w| w.tsh0()._16().tsh1()._16().tsh2()._16().tsh3()._16());
                    adc_registers.sstsh2.write(|w| w.tsh0()._16().tsh1()._16().tsh2()._16().tsh3()._16());
                    adc_registers.sstsh3.write(|w| w.tsh0()._16());
                },
                _32x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._32().tsh1()._32().tsh2()._32().tsh3()._32().tsh4()._32().tsh5()._32().tsh0()._32().tsh7()._32());
                    adc_registers.sstsh1.write(|w| w.tsh0()._32().tsh1()._32().tsh2()._32().tsh3()._32());
                    adc_registers.sstsh2.write(|w| w.tsh0()._32().tsh1()._32().tsh2()._32().tsh3()._32());
                    adc_registers.sstsh3.write(|w| w.tsh0()._32());
                },
                _64x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._64().tsh1()._64().tsh2()._64().tsh3()._64().tsh4()._64().tsh5()._64().tsh0()._64().tsh7()._64());
                    adc_registers.sstsh1.write(|w| w.tsh0()._64().tsh1()._64().tsh2()._64().tsh3()._64());
                    adc_registers.sstsh2.write(|w| w.tsh0()._64().tsh1()._64().tsh2()._64().tsh3()._64());
                    adc_registers.sstsh3.write(|w| w.tsh0()._64());
                },
                _128x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._128().tsh1()._128().tsh2()._128().tsh3()._128().tsh4()._128().tsh5()._128().tsh0()._128().tsh7()._128());
                    adc_registers.sstsh1.write(|w| w.tsh0()._128().tsh1()._128().tsh2()._128().tsh3()._128());
                    adc_registers.sstsh2.write(|w| w.tsh0()._128().tsh1()._128().tsh2()._128().tsh3()._128());
                    adc_registers.sstsh3.write(|w| w.tsh0()._128());
                },
                _256x => {
                    adc_registers.sstsh0.write(|w| w.tsh0()._256().tsh1()._256().tsh2()._256().tsh3()._256().tsh4()._256().tsh5()._256().tsh0()._256().tsh7()._256());
                    adc_registers.sstsh1.write(|w| w.tsh0()._256().tsh1()._256().tsh2()._256().tsh3()._256());
                    adc_registers.sstsh2.write(|w| w.tsh0()._256().tsh1()._256().tsh2()._256().tsh3()._256());
                    adc_registers.sstsh3.write(|w| w.tsh0()._256());
                },
            }
        }

        // Use main oscillator as ADC clock
        adc_registers.cc.write(|w| w.cs().mosc());

        //
        // 4. Enable DMA then ADC
        //
        adc_registers.actss.write(|w| {
            w.aden0()
                .set_bit()
                .aden1()
                .set_bit()
                .aden2()
                .set_bit()
                .aden3()
                .set_bit()
        }); // Must be enabled before adc or it will crash with no error
        adc_registers.actss.write(|w| {
            w.asen0()
                .set_bit()
                .asen1()
                .set_bit()
                .asen2()
                .set_bit()
                .asen3()
                .set_bit()
        }); // Must be configured before enabling or it will crash with no error

        Self {
            registers: adc_registers,
            _gpio_portb: gpiob,
            _gpio_portd: gpiod,
            _gpio_porte: gpioe,
            _gpio_portk: gpiok,
        }
    }
}


//! Number of samples to take and average in hardware
//! per sample delivered to the FIFO
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


//! Number of ADC clock beats to hold each sample
#[repr(u8)]
pub enum SampleHoldMultiplier {
    _4x,
    _8x,
    _16x,
    _32x,
    _64x,
    _128x,
    _256x
}