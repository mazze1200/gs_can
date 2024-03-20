#![no_std]
#![no_main]

use core::mem;

use core::pin::pin;
use defmt::{panic, *};
use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::can::frame::FdFrame;
use embassy_stm32::can::{FdcanControl, FdcanRx, FdcanTxEvent, Instance};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_time::{Instant, Timer};

use embassy_stm32::peripherals::*;
use embassy_stm32::can;
use embassy_usb::Builder;
use futures::future::join5;
use futures::stream::{select, unfold};
use futures::StreamExt;
use gs_can::{GsDeviceBittiming, HostFrame};
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;

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
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
                }
                1 => {
                    self.can_cnt_1.into_config_mode();
                    self.can_cnt_1.set_bitrate(bit_timing);
                    self.can_cnt_1
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
                }
                2 => {
                    self.can_cnt_2.into_config_mode();
                    self.can_cnt_2.set_bitrate(bit_timing);
                    self.can_cnt_2
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
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
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
                }
                1 => {
                    self.can_cnt_1.into_config_mode();
                    self.can_cnt_1.set_fd_data_bitrate(data_bit_timing);
                    self.can_cnt_1
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
                }
                2 => {
                    self.can_cnt_2.into_config_mode();
                    self.can_cnt_2.set_fd_data_bitrate(data_bit_timing);
                    self.can_cnt_2
                        .start(can::FdcanOperatingMode::InternalLoopbackMode);
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
            /*
                .feature =
                    GS_CAN_FEATURE_LISTEN_ONLY |
                    GS_CAN_FEATURE_LOOP_BACK |
                    GS_CAN_FEATURE_HW_TIMESTAMP |
                    GS_CAN_FEATURE_IDENTIFY |
                    GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE
            */

            // timing.set_features(
            //     GsDeviceBtConstFeature::GsCanFeatureFd
            //         | GsDeviceBtConstFeature::GsCanFeatureBtConstExt
            //         | GsDeviceBtConstFeature::GsCanFeatureHwTimestamp
            //         | GsDeviceBtConstFeature::GsCanFeatureListenOnly,
            // );

            timing.set_features(
                GsDeviceBtConstFeature::GsCanFeatureFd
                    | GsDeviceBtConstFeature::GsCanFeatureBtConstExt
                    | GsDeviceBtConstFeature::GsCanFeatureListenOnly
                    | GsDeviceBtConstFeature::GsCanFeatureHwTimestamp
                    | GsDeviceBtConstFeature::GsCanFeatureLoopBack
                    | GsDeviceBtConstFeature::GsCanFeatureIdentify
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

            /*
                        .tseg1_min = 1,
            .tseg1_max = 16,
            .tseg2_min = 1,
            .tseg2_max = 8,
            .sjw_max = 4,
            .brp_min = 1,
            .brp_max = 1024,
            .brp_inc = 1,
                     */
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

pub enum Event {
    CanRx(embassy_stm32::can::frame::FdFrame, Instant, u8),
    CanTx(u32, Instant, u8),
    UsbRx(HostFrame),
}

fn create_can_rx_stream<'d, I: Instance>(
    can: FdcanRx<'d, I>,
    can_index: u8,
) -> impl futures::Stream<Item = Event> + 'd {
    unfold((can, can_index), |(mut rx, index)| async move {
        loop {
            let res = rx.read_fd().await;
            match res {
                Ok((msg, ts)) => return Some((Event::CanRx(msg, ts, index), (rx, index))),
                Err(_) => {}
            }
        }
    })
}

fn create_can_tx_event_stream<I: Instance>(
    can: FdcanTxEvent<I>,
    can_index: u8,
) -> impl futures::Stream<Item = Event> {
    unfold((can, can_index), |(mut rx, index)| async move {
        let (_header, marker, timestamp) = rx.read_tx_event().await;
        Some((Event::CanTx(marker as u32, timestamp, index), (rx, index)))
    })
}

static CAN_HANDLER: StaticCell<CanHandler> = StaticCell::new();
static GS_HOST_FRAMES: StaticCell<[[Option<HostFrame>; 10]; 3]> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

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

    // create can
    let mut can0 = can::FdcanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can0.set_bitrate(500_000);
    can0.set_fd_data_bitrate(4_000_000, true);
    let can0 = can0.into_internal_loopback_mode();
    let (mut can_tx_0, can_rx_0, can_tx_event_0, can_cnt_0) = can0.split_with_control();

    let mut can1 = can::FdcanConfigurator::new(p.FDCAN2, p.PB5, p.PB6, Irqs);
    can1.set_bitrate(500_000);
    can1.set_fd_data_bitrate(4_000_000, true);
    let can1 = can1.into_internal_loopback_mode();
    let (mut can_tx_1, can_rx_1, can_tx_event_1, can_cnt_1) = can1.split_with_control();

    let mut can2 = can::FdcanConfigurator::new(p.FDCAN3, p.PG10, p.PG9, Irqs);
    can2.set_bitrate(500_000);
    can2.set_fd_data_bitrate(4_000_000, true);
    let can2 = can2.into_internal_loopback_mode();
    let (mut can_tx_2, can_rx_2, can_tx_event_2, can_cnt_2) = can2.split_with_control();

    let can_rx_stream_0 = create_can_rx_stream(can_rx_0, 0);
    let can_rx_stream_1 = create_can_rx_stream(can_rx_1, 1);
    let can_rx_stream_2 = create_can_rx_stream(can_rx_2, 2);

    let can_tx_event_stream_0 = create_can_tx_event_stream(can_tx_event_0, 0);
    let can_tx_event_stream_1 = create_can_tx_event_stream(can_tx_event_1, 1);
    let can_tx_event_stream_2 = create_can_tx_event_stream(can_tx_event_2, 2);

    let can_tx_channel = Channel::<NoopRawMutex, (FdFrame, u8, u8), 6>::new();

    let can_tx_fut = async {
        loop {
            let (frame, channel, echo_id) = can_tx_channel.receive().await;
            let tx_res = match channel {
                0 => can_tx_0.write_fd_with_marker(&frame, Some(echo_id)).await,
                1 => can_tx_1.write_fd_with_marker(&frame, Some(echo_id)).await,
                2 => can_tx_2.write_fd_with_marker(&frame, Some(echo_id)).await,
                _ => {
                    warn!("Invalid CAN channel {}", channel);
                    None
                }
            };

            if tx_res.is_some() {
                warn!("Add error handling for full buffer!");
            }
        }
    };

    let can_handler = CAN_HANDLER.init(CanHandler {
        can_cnt_0,
        can_cnt_1,
        can_cnt_2,
    });

    info!("CAN Configured");

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
    config.manufacturer = Some("Mazze");
    config.product = Some("GS_CAN");
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
    let mut control_buf = [0; 128];

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

    // Create classes on the builder.
    let class = GsCanClass::new(&mut builder, &mut state, 3, can_handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let (mut usb_tx, usb_rx) = class.split();

    let usb_rx = pin!(stream::unfold(usb_rx, |mut usb_rx| async move {
        usb_rx.wait_connection().await;
        info!("USB RX connected");
        let frame = usb_rx.read_frame().await;
        info!("Read Frame from USB");
        match frame {
            Ok(frame) => return Some((Event::UsbRx(frame), usb_rx)),
            Err(error) => warn!("Invalid Frame: Error {:?}", error),
        };

        None
    }));

    let usb_tx_channel = Channel::<NoopRawMutex, HostFrame, 6>::new();
    let usb_tx_fut = async {
        loop {
            usb_tx.wait_connection().await;
            info!("USB TX connected");

            let frame = usb_tx_channel.receive().await;
            let tx_res = usb_tx.write_frame(&frame).await;
            if tx_res.is_err() {
                warn!("Add error handling!");
            }
        }
    };

    info!("USB Configured");

    let mut selectors = pin!(select(
        select(
            select(can_rx_stream_0, can_rx_stream_1),
            select(can_tx_event_stream_0, can_tx_event_stream_1)
        ),
        select(select(can_rx_stream_2, can_tx_event_stream_2), usb_rx)
    ));

    let host_frames =
        GS_HOST_FRAMES.init(array_init::array_init(|_| array_init::array_init(|_| None)));

    let main_handler = async {
        while let Some(event) = selectors.next().await {
            match event {
                Event::CanRx(frame, ts, channel) => {
                    info!("CanRx {}", ts);
                    let host_frame =
                    HostFrame::new_from(&frame, channel, -1i32 as u32, ts.as_micros() as u32);

                    usb_tx_channel.send(host_frame).await;
                }
                Event::UsbRx(frame) => {
                    info!("Can Host Frame received");

                    let can_frame: FdFrame = (&frame).into();

                    can_tx_channel
                        .send((can_frame, frame.get_channel(), (frame.get_echo_id() as u8)))
                        .await;

                    let channel = frame.get_channel() as usize;
                    let echo_id = frame.get_echo_id() as usize;
                    if let Some(channel_host_frames) = host_frames.get_mut(channel) {
                        if let Some(host_frame) = channel_host_frames.get_mut(echo_id) {
                            *host_frame = Some(frame);
                            continue;
                        }
                    }

                    warn!("Add error handling!");
                }
                Event::CanTx(echo_id, ts, channel) => {
                    info!("CanTx {}", echo_id);

                    if let Some(channel_host_frames) = host_frames.get_mut(channel as usize) {
                        if let Some(host_frame) = channel_host_frames.get_mut((echo_id) as usize) {
                            let frame = mem::take(host_frame);
                            if let Some(mut frame) = frame {
                                frame.set_timestamp(ts.as_micros() as u32);

                                usb_tx_channel.send(frame).await;
                            } else {
                                warn!("here should be a frame but isn't!");
                            }

                            continue;
                        }
                    }

                    warn!("Add error handling!");
                }
            }
        }
    };

    let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    let led_fut = async {
        loop {
            // info!("LED high");
            led.set_high();
            Timer::after_millis(500).await;

            // info!("LED low");
            led.set_low();
            Timer::after_millis(500).await;
        }
    };

    info!("Start handlers");

    join5(led_fut, usb_fut, main_handler, can_tx_fut, usb_tx_fut).await;
}
