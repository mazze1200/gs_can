#![no_std]
#![no_main]

use core::ops::IndexMut;
use core::pin::pin;

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::frame::{self, ClassicFrame};
use embassy_stm32::can::{CanFrame, FdcanControl, FdcanRx, Timestamp};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_time::{Instant, Timer};

use embassy_stm32::peripherals::*;
use embassy_stm32::{can, rcc};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use futures::future::{join, join3};
use futures::stream::select;
use futures::StreamExt;
use gs_can::{GsDeviceBittiming, GsDeviceBtConstFeatures, GsHostFrame};
use static_cell::StaticCell;
use zerocopy::{FromZeroes, Ref};

use futures::prelude::*;

use crate::gs_can::{GsCanClass, GsCanHandlers, GsDeviceBtConstFeature, State};

use {defmt_rtt as _, panic_probe as _};

mod gs_can;

bind_interrupts!(struct Irqs {
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
    FDCAN3_IT0 => can::IT0InterruptHandler<FDCAN3>;
    FDCAN3_IT1 => can::IT1InterruptHandler<FDCAN3>;
});

struct CanHandler {
    // cans: [FdcanControl; 3]
    can_cnt_0: FdcanControl<FDCAN1>,
    can_cnt_1: FdcanControl<FDCAN2>,
    can_cnt_2: FdcanControl<FDCAN3>,
}

impl GsCanHandlers for CanHandler {
    fn get_timestamp(&self) -> embassy_time::Instant {
        embassy_time::Instant::now()
    }

    fn set_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming) {
        if let Some(bit_timing) = Into::<Option<can::config::NominalBitTiming>>::into(timing) {
            match channel {
                0 => {
                    self.can_cnt_0.into_config_mode();
                    self.can_cnt_0.set_bitrate(bit_timing);
                    self.can_cnt_0
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                1 => {
                    self.can_cnt_1.into_config_mode();
                    self.can_cnt_1.set_bitrate(bit_timing);
                    self.can_cnt_1
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                2 => {
                    self.can_cnt_2.into_config_mode();
                    self.can_cnt_2.set_bitrate(bit_timing);
                    self.can_cnt_2
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                _ => {}
            };
        }
    }

    fn set_data_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming) {
        if let Some(data_bit_timing) = Into::<Option<can::config::DataBitTiming>>::into(timing) {
            match channel {
                0 => {
                    self.can_cnt_0.into_config_mode();
                    self.can_cnt_0.set_fd_data_bitrate(data_bit_timing);
                    self.can_cnt_0
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                1 => {
                    self.can_cnt_1.into_config_mode();
                    self.can_cnt_1.set_fd_data_bitrate(data_bit_timing);
                    self.can_cnt_1
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                2 => {
                    self.can_cnt_2.into_config_mode();
                    self.can_cnt_2.set_fd_data_bitrate(data_bit_timing);
                    self.can_cnt_2
                        .start(can::FdcanOperatingMode::NormalOperationMode);
                }
                _ => {}
            }
        };
    }

    fn get_bittiming(&self, channel: u16, timing: &mut gs_can::GsDeviceBtConst) {
        if let Some(frequency) = match channel {
            0 => Some(self.can_cnt_0.get_frequency()),
            1 => Some(self.can_cnt_1.get_frequency()),
            2 => Some(self.can_cnt_2.get_frequency()),
            _ => None,
        } {
            timing.set_features(
                GsDeviceBtConstFeature::GsCanFeatureFd
                    | GsDeviceBtConstFeature::GsCanFeatureBtConstExt
                    | GsDeviceBtConstFeature::GsCanFeatureHwTimestamp
                    | GsDeviceBtConstFeature::GsCanFeatureListenOnly,
            );
            timing.fclk_can.set(frequency.0);
            timing.tseg1_min.set(1);
            timing.tseg1_max.set(255);
            timing.tseg2_min.set(1);
            timing.tseg2_max.set(127);
            timing.sjw_max.set(127);
            timing.brp_min.set(1);
            timing.brp_max.set(511);
            timing.brp_inc.set(1);
        }
    }

    fn get_bittiming_extended(&self, channel: u16, timing: &mut gs_can::GsDeviceBtConstExtended) {
        if let Some(frequency) = match channel {
            0 => Some(self.can_cnt_0.get_frequency()),
            1 => Some(self.can_cnt_1.get_frequency()),
            2 => Some(self.can_cnt_2.get_frequency()),
            _ => None,
        } {
            timing.set_features(
                GsDeviceBtConstFeature::GsCanFeatureFd
                    | GsDeviceBtConstFeature::GsCanFeatureBtConstExt
                    | GsDeviceBtConstFeature::GsCanFeatureHwTimestamp
                    | GsDeviceBtConstFeature::GsCanFeatureListenOnly,
            );
            timing.fclk_can.set(frequency.0);
            timing.tseg1_min.set(1);
            timing.tseg1_max.set(255);
            timing.tseg2_min.set(1);
            timing.tseg2_max.set(127);
            timing.sjw_max.set(127);
            timing.brp_min.set(1);
            timing.brp_max.set(511);
            timing.brp_inc.set(1);

            timing.dtseg1_min.set(1);
            timing.dtseg1_max.set(31);
            timing.dtseg2_min.set(1);
            timing.dtseg2_max.set(15);
            timing.dsjw_max.set(15);
            timing.dbrp_min.set(1);
            timing.dbrp_max.set(31);
            timing.dbrp_inc.set(1);
        }
    }
}

static CAN_HANDLER: StaticCell<CanHandler> = StaticCell::new();

pub enum Event {
    CanRx(embassy_stm32::can::CanFrame, Instant, u8),
    CanTx(u32, u8),
    UsbRx(GsHostFrame),
}

// static GS_HOST_FRAMES_0: StaticCell<[GsHostFrame; 10]> = StaticCell::new();
// static GS_HOST_FRAMES_1: StaticCell<[GsHostFrame; 10]> = StaticCell::new();
// static GS_HOST_FRAMES_2: StaticCell<[GsHostFrame; 10]> = StaticCell::new();

static GS_HOST_FRAMES: StaticCell<[[Option<GsHostFrame>; 10]; 3]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
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
    config.rcc.hse = Some(rcc::Hse {
        freq: embassy_stm32::time::Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });
    config.rcc.mux.fdcansel = rcc::mux::Fdcansel::HSE;

    let p = embassy_stm32::init(config);

    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = true;
    let driver = Driver::new_fs(
        p.USB_OTG_HS,
        Irqs,
        p.PA12,
        p.PA11,
        &mut ep_out_buffer,
        config,
    );

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x1d50, 0x606f);
    config.manufacturer = Some("Embassy");
    config.product = Some("GS_CAN example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xff;
    config.device_sub_class = 0xff;
    config.device_protocol = 0xff;
    config.composite_with_iads = false;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut device_descriptor = [0; 256];
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut device_descriptor,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // create can
    let mut can0 = can::FdcanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can0.set_bitrate(500_000);
    can0.set_fd_data_bitrate(4_000_000, true);
    let can0 = can0.into_internal_loopback_mode();
    let (mut can_tx_0, mut can_rx_0, can_cnt_0) = can0.split_with_control();

    let mut can1 = can::FdcanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);
    can1.set_bitrate(500_000);
    can1.set_fd_data_bitrate(4_000_000, true);
    let mut can1 = can1.into_internal_loopback_mode();
    let (can_tx_1, can_rx_1, can_cnt_1) = can1.split_with_control();

    let mut can2 = can::FdcanConfigurator::new(p.FDCAN3, p.PG10, p.PG9, Irqs);
    can2.set_bitrate(500_000);
    can2.set_fd_data_bitrate(4_000_000, true);
    let mut can2 = can2.into_internal_loopback_mode();
    let (can_tx_2, can_rx_2, can_cnt_2) = can2.split_with_control();

    // for data futures::stream::select(can_rx_0,can_rx_1);

    // let v = can_rx_0.map(|x| x).next();
    // let data =  can_rx_0.next().map(|v| v).await;

    let can_rx_0 = can_rx_0.map(|(frame, ts)| Event::CanRx(frame, ts, 0));
    let can_rx_1 = can_rx_1.map(|(frame, ts)| Event::CanRx(frame, ts, 1));
    let can_rx_2 = can_rx_2.map(|(frame, ts)| Event::CanRx(frame, ts, 2));

    info!("CAN Configured");

    let can_handler = CAN_HANDLER.init(CanHandler {
        can_cnt_0: can_cnt_0,
        can_cnt_1: can_cnt_1,
        can_cnt_2: can_cnt_2,
    });

    // Create classes on the builder.
    let mut class = GsCanClass::new(&mut builder, &mut state, 3, can_handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    /// Control not needed, right?
    // let (usb_tx, usb_rx, usb_control) = class.split_with_control();
    let (usb_tx, usb_rx) = class.split();

    let usb_rx = pin!(stream::unfold(usb_rx, |mut usb_rx| async move {
        let frame = usb_rx.read_frame().await;
        if let Ok(frame) = frame {
            return Some((Event::UsbRx(frame), usb_rx));
        }

        None
    }));

    let mut selectors = select(select(can_rx_0, can_rx_1), select(can_rx_2, usb_rx));

    // let host_frames_0 =
    //     GS_HOST_FRAMES_0.init(array_init::array_init(|_| GsHostFrame::new_zeroed()));
    // let host_frames_1 =
    //     GS_HOST_FRAMES_1.init(array_init::array_init(|_| GsHostFrame::new_zeroed()));
    // let host_frames_2 =
    //     GS_HOST_FRAMES_2.init(array_init::array_init(|_| GsHostFrame::new_zeroed()));

    let host_frames = GS_HOST_FRAMES.init(array_init::array_init(|_| {
        // array_init::array_init(|_| GsHostFrame::new_zeroed())
        array_init::array_init(|_| None)
    }));

    let my_frame = GsHostFrame::new_zeroed();

    if let Some(hfs) = host_frames.get_mut(2) {
        hfs[12] = Some(my_frame);
        // if let Some(frames) = hfs{
        //     // frames[0] = Some(my_frame);
        // }
    }

    // let r = &host_frames[4];

    let main_handler = async {
        while let Some(event) = selectors.next().await {
            match event {
                Event::CanRx(frame, ts, channel) => info!("{}", ts),
                Event::UsbRx(frame) => {
                    info!("Can Host Frame received");

                    let can_frame: CanFrame = (&frame).into();

                    let tx_res = &mut can_tx_0.write_frame(&can_frame).await;

                    let channel = frame.channel as usize;
                    let echo_id = frame.echo_id.get() as usize;
                    if let Some(channel_host_frames) = host_frames.get_mut(channel) {
                        if let Some(host_frame) = channel_host_frames.get_mut(echo_id) {
                            *host_frame = Some(frame);
                        }
                    }
                }
                Event::CanTx(echo_id, can) => info!("{}", echo_id),
            }
        }
    };

    let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    let led_fut = async {
        loop {
            info!("high");
            led.set_high();
            Timer::after_millis(500).await;

            info!("low");
            led.set_low();
            Timer::after_millis(500).await;
        }
    };

    join3(led_fut, usb_fut, main_handler).await;
}
