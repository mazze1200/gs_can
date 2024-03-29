#![no_std]
#![no_main]

use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::time::{hz, khz};
use embassy_stm32::timer::low_level::CoreInstance;
use embedded_can::ExtendedId;
use futures::future::{join, join4};

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::can::frame::Header;
use embassy_stm32::peripherals::*;
use embassy_stm32::rcc::mux;
use embassy_stm32::{bind_interrupts, can, rcc, Config};
use embassy_time::Timer;
use num_traits::float;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
    FDCAN3_IT0 => can::IT0InterruptHandler<FDCAN3>;
    FDCAN3_IT1 => can::IT1InterruptHandler<FDCAN3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello CAN!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1); // 64 Mhz
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,   // 64 Mhz
            prediv: PllPreDiv::DIV4,  // 16 Mhz, this has to be between 2 Mhz and 16 Mhz
            mul: PllMul::MUL48,       // 768 Mhz
            divp: Some(PllDiv::DIV2), // 384 Mhz, these dividers have to be at least 2 to create a 50/50 duty cycle (the VCO does not guarantee that)
            divq: Some(PllDiv::DIV8), // 96 Mhz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 384 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 192 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 96 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 96 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 96 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 96 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);

    let tim3 = p.TIM3;
    TIM3::enable_and_reset();

    let core_freq = embassy_stm32::peripherals::TIM3::frequency().0;
    info!("Timer core Freq: {}", core_freq);

    let regs = TIM3::regs_core();

    // The PSC is a devider for the timer core clock. The hardware automatically adds +1.
    regs.psc().modify(|r| r.set_psc(191));
    regs.arr().modify(|r| r.set_arr(0xffff));
    regs.cr1()
        .modify(|r| r.set_urs(embassy_stm32::pac::timer::vals::Urs::COUNTERONLY));
    regs.egr().write(|r| r.set_ug(true));

    let r = <embassy_stm32::peripherals::TIM3 as embassy_stm32::timer::low_level::GeneralPurpose16bitInstance>::regs_gp16();
    r.ccmr_output(0).modify(|w| {
        w.set_ocm(0, embassy_stm32::timer::OutputCompareMode::Frozen.into());
        w.set_ocm(1, embassy_stm32::timer::OutputCompareMode::Frozen.into());
    });

    r.ccmr_output(1).modify(|w| {
        w.set_ocm(0, embassy_stm32::timer::OutputCompareMode::Frozen.into());
        w.set_ocm(1, embassy_stm32::timer::OutputCompareMode::Frozen.into());
    });

    tim3.start();

    // create can
    let mut can0 = can::FdcanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can0.set_bitrate(500_000);
    can0.set_fd_data_bitrate(4_000_000, true);
    let can0 = can0.into_normal_mode();
    let (mut can_tx_0, mut can_rx_0, _can_tx_event_0, _can_cnt_0) = can0.split_with_control();

    let mut can1 = can::FdcanConfigurator::new(p.FDCAN2, p.PB12, p.PB6, Irqs);
    can1.set_bitrate(500_000);
    can1.set_fd_data_bitrate(4_000_000, true);
    let can1 = can1.into_normal_mode();
    let (_can_tx_1, mut can_rx_1, _can_tx_event_1, _can_cnt_1) = can1.split_with_control();

    let mut can2 = can::FdcanConfigurator::new(p.FDCAN3, p.PF6, p.PF7, Irqs);
    can2.set_bitrate(500_000);
    can2.set_fd_data_bitrate(4_000_000, true);
    let can2 = can2.into_normal_mode();
    let (_can_tx_2, mut can_rx_2, _can_tx_event_2, _can_cnt_2) = can2.split_with_control();

    info!("CAN Configured");

    let tx_loop = async {
        let mut i = 0u8;
        // let mut last_read_ts = embassy_time::Instant::now();

        info!("Starting TX loop");
        loop {
            let frame = can::frame::FdFrame::new(
                Header::new_fd(
                    embedded_can::Id::Extended(ExtendedId::new(1234).unwrap()),
                    64,
                    false,
                    true,
                ),
                &[i; 64],
            )
            .unwrap();
            // info!("Writing frame");
            _ = can_tx_0.write_fd(&frame).await;

            Timer::after_millis(1000).await;

            i = i.overflowing_add(1u8).0;
        }
    };

    let rx_loop0 = async {
        info!("Start RX loop");
        loop {
            match can_rx_0.read_fd().await {
                Ok((rx_frame, ts)) => {
                    info!(
                        "CAN 0 Rx: {:x} {:x} {:x}",
                        rx_frame.data().len(),
                        rx_frame.data()[0],
                        ts.as_micros() as f64 / 1_000_000.0,
                    )
                }
                Err(_err) => error!("Error in frame"),
            }

            // info!("Timer Frequency: {}", tim3.get_frequency());
        }
    };

    let rx_loop1 = async {
        info!("Start RX loop");
        loop {
            match can_rx_1.read_fd().await {
                Ok((rx_frame, ts)) => {
                    info!("CAN 1 Rx:{:x}", ts.as_micros() as f64 / 1_000_000.0,)
                }
                Err(_err) => error!("Error in frame"),
            }

            // info!("Timer Frequency: {}", tim3.get_frequency());
        }
    };

    let rx_loop2 = async {
        info!("Start RX loop");
        loop {
            match can_rx_2.read_fd().await {
                Ok((rx_frame, ts)) => {
                    info!("CAN 2 Rx:{:x}", ts.as_micros() as f64 / 1_000_000.0,)
                }
                Err(_err) => error!("Error in frame"),
            }

            // info!("Timer Frequency: {}", tim3.get_frequency());
        }
    };

    let _ = join4(rx_loop0, rx_loop1, rx_loop2, tx_loop).await;
}
