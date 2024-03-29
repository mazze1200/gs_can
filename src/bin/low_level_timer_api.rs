#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::low_level::AFType;
use embassy_stm32::gpio::Speed;
use embassy_stm32::time::{khz, Hertz};
use embassy_stm32::timer::*;
use embassy_stm32::{into_ref, Config, Peripheral, PeripheralRef};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // 100mhz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    let p = embassy_stm32::init(config);

    info!("Hello World!");

    let tim3 = p.TIM3;

    // let mut pwm_2 = SimplePwm32::new(, p.PC8, khz(10));

    let mut pwm = SimplePwm32::new(tim3.into_ref(), p.PB0, khz(10));
    let max = pwm.get_max_duty();
    pwm.enable(Channel::Ch3);

    info!("PWM initialized");
    info!("PWM max duty {}", max);

    loop {
        info!("Reload");

        pwm.set_duty(Channel::Ch3, 0);
        Timer::after_millis(300).await;
        pwm.set_duty(Channel::Ch3, max / 4);
        Timer::after_millis(300).await;
        pwm.set_duty(Channel::Ch3, max / 2);
        Timer::after_millis(300).await;
        pwm.set_duty(Channel::Ch3, max - 1);
        Timer::after_millis(300).await;
    }
}
pub struct SimplePwm32<'d, T: CaptureCompare16bitInstance> {
    inner: PeripheralRef<'d, T>,
}

impl<'d, T: CaptureCompare16bitInstance> SimplePwm32<'d, T> {
    pub fn new(
        tim: impl Peripheral<P = T> + 'd,
        ch3: impl Peripheral<P = impl Channel3Pin<T>> + 'd,
        freq: Hertz,
    ) -> Self {
        into_ref!(tim, ch3);

        T::enable_and_reset();

        ch3.set_speed(Speed::VeryHigh);
        ch3.set_as_af(ch3.af_num(), AFType::OutputPushPull);

        let this = Self { inner: tim };

        this.inner.set_frequency(freq);
        this.inner.set_autoreload_preload(true);
        this.inner.start();

        let r = T::regs_gp16();
        // r.ccmr_output(0)
        //     .modify(|w| w.set_ocm(0, OutputCompareMode::PwmMode1.into()));
        // r.ccmr_output(0)
        //     .modify(|w| w.set_ocm(1, OutputCompareMode::PwmMode1.into()));

        // This enables the output for channel 3
        r.ccmr_output(1)
            .modify(|w| w.set_ocm(0, OutputCompareMode::PwmMode1.into()));

        // r.ccmr_output(1)
        //     .modify(|w| w.set_ocm(1, OutputCompareMode::PwmMode1.into()));

        this
    }

    pub fn enable(&mut self, channel: Channel) {
        T::regs_gp16()
            .ccer()
            .modify(|w| w.set_cce(channel.index(), true));
    }

    pub fn disable(&mut self, channel: Channel) {
        T::regs_gp16()
            .ccer()
            .modify(|w| w.set_cce(channel.index(), false));
    }

    pub fn get_max_duty(&self) -> u16 {
        T::regs_gp16().arr().read().arr()
    }

    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        defmt::assert!(duty < self.get_max_duty());
        T::regs_gp16()
            .ccr(channel.index())
            .write(|val| val.set_ccr(duty));
    }
}
