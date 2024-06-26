#![no_std]
#![no_main]

use core::mem;

use core::pin::pin;
use defmt::{panic, *};
use defmt_rtt as _;
use embassy_executor::Spawner;

use embassy_stm32::can::frame::FdFrame;
use embassy_stm32::can::{FdCanConfiguration, FdcanRx, FdcanTxEvent, Instance};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::timer::low_level::CoreInstance;
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb_otg, Config};
use embassy_time::{Instant, Timer};

use embassy_stm32::can;
use embassy_stm32::peripherals::*;
use embassy_usb::{Builder, UsbDevice};
use futures::future::{join3, join4};
use futures::stream::{select, unfold};
use futures::StreamExt;
use gs_can::HostFrame;
use static_cell::StaticCell;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;

use futures::prelude::*;

use crate::gs_can::{GsCanClass, State};

use {defmt_rtt as _, panic_probe as _};

mod can_control_handler;
mod gs_can;

use can_control_handler::CanControlHandler;

bind_interrupts!(struct Irqs {
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;
    FDCAN3_IT0 => can::IT0InterruptHandler<FDCAN3>;
    FDCAN3_IT1 => can::IT1InterruptHandler<FDCAN3>;
    // TIM3 => timer::InterruptHandler;
});

type UsbDriver = Driver<'static, embassy_stm32::peripherals::USB_OTG_HS>;

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

pub enum Event {
    /// Frame, Timestamp, Channel
    CanRx(embassy_stm32::can::frame::FdFrame, Instant, u8),
    /// echo_id, Timestamp, Channel
    CanTx(u8, Instant, u8),
    /// Frame
    UsbRx(HostFrame),
    /// CAN bus error
    /// Channel index
    CanErr(u8),
}

fn create_can_rx_stream<'d, I: Instance>(
    can: FdcanRx<'d, I>,
    can_channel: u8,
) -> impl futures::Stream<Item = Event> + 'd {
    unfold((can, can_channel), |(mut rx, can_channel)| async move {
        // loop
        {
            match rx.read_fd().await {
                Ok((frame, ts)) => {
                    trace!(
                        "Received CAN Frame |  Channel: {}, Header: {}",
                        can_channel,
                        frame.header()
                    );
                    return Some((Event::CanRx(frame, ts, can_channel), (rx, can_channel)));
                }
                Err(bus_error) => {
                    trace!("handle CAN Bus errors: {}", bus_error);
                    match bus_error {
                        can::enums::BusError::BusOff => {
                            I::registers().regs.cccr().modify(|r| r.set_init(false));
                            Timer::after_millis(10).await;
                            return Some((Event::CanErr(can_channel), (rx, can_channel)));
                        }
                        can::enums::BusError::BusPassive => {
                            Timer::after_millis(1).await;
                            return Some((Event::CanErr(can_channel), (rx, can_channel)));
                        }
                        can::enums::BusError::BusWarning => {
                            Timer::after_millis(1).await;
                            return Some((Event::CanErr(can_channel), (rx, can_channel)));
                        }
                        _ => {
                            return Some((Event::CanErr(can_channel), (rx, can_channel)));
                        }
                    }
                }
            }
        }
    })
}

fn create_can_tx_event_stream<I: Instance>(
    can: FdcanTxEvent<I>,
    can_channel: u8,
) -> impl futures::Stream<Item = Event> {
    unfold((can, can_channel), |(mut rx, can_channel)| async move {
        let (_header, marker, timestamp) = rx.read_tx_event().await;
        Some((
            Event::CanTx(marker, timestamp, can_channel),
            (rx, can_channel),
        ))
    })
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1); // 64 MHz
        config.rcc.csi = true;
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        }); // needed for USB
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,   // 64 MHz
            prediv: PllPreDiv::DIV4,  // 16 MHz, this has to be between 2 MHz and 16 MHz
            mul: PllMul::MUL48,       // 768 MHz
            divp: Some(PllDiv::DIV2), // 384 MHz, these dividers have to be at least 2 to create a 50/50 duty cycle (the VCO does not guarantee that)
            divq: Some(PllDiv::DIV8), // 96 MHz
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 384 MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 192 MHz, also the clock for timer 3 (TIM3)
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 96 MHz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.fdcansel = mux::Fdcansel::PLL1_Q; // 96 MHz
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }

    let p = embassy_stm32::init(config);

    // Config TIM3 as a time base for FDCAN timestamps
    let tim3 = p.TIM3;
    TIM3::enable_and_reset();

    let core_freq = TIM3::frequency().0;
    debug!("Timer core Freq: {}", core_freq);

    let regs = <TIM3 as embassy_stm32::timer::low_level::GeneralPurpose16bitInstance>::regs_gp16();

    // Since the TIM3 core clock is equal to AHB clock (192 MHz) we want to devide the core clock by 192 to get a 1 MHz FDCAN timestamp clock base.
    // The PSC is a devider for the timer core clock. The hardware automatically adds +1.
    regs.psc().modify(|r| r.set_psc(191));
    regs.arr().modify(|r| r.set_arr(0xffff));
    regs.cr1()
        .modify(|r| r.set_urs(embassy_stm32::pac::timer::vals::Urs::COUNTERONLY));
    regs.egr().write(|r| r.set_ug(true));

    regs.ccmr_output(0).modify(|w| {
        w.set_ocm(0, embassy_stm32::timer::OutputCompareMode::Frozen.into());
        w.set_ocm(1, embassy_stm32::timer::OutputCompareMode::Frozen.into());
    });

    regs.ccmr_output(1).modify(|w| {
        w.set_ocm(0, embassy_stm32::timer::OutputCompareMode::Frozen.into());
        w.set_ocm(1, embassy_stm32::timer::OutputCompareMode::Frozen.into());
    });

    tim3.start();

    // create can
    let mut can0 = can::FdcanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    can0.set_bitrate(500_000);
    can0.set_fd_data_bitrate(1_000_000, true);
    can0.set_tx_mode(can::config::TxBufferMode::Fifo);
    let can0 = can0.into_normal_mode();
    let (mut can_tx_0, can_rx_0, can_tx_event_0, can_cnt_0) = can0.split_with_control();

    debug!(
        "CAN 0 Bit Timing {}",
        can_cnt_0.get_config().get_nominal_bit_timing()
    );
    debug!(
        "CAN 0 Data Bit Timing {}",
        can_cnt_0.get_config().get_data_bit_timing()
    );

    let mut can1 = can::FdcanConfigurator::new(p.FDCAN2, p.PB12, p.PB6, Irqs);
    can1.set_bitrate(500_000);
    can1.set_fd_data_bitrate(1_000_000, true);
    can1.set_tx_mode(can::config::TxBufferMode::Fifo);
    let can1 = can1.into_normal_mode();
    let (mut can_tx_1, can_rx_1, can_tx_event_1, can_cnt_1) = can1.split_with_control();

    let mut can2 = can::FdcanConfigurator::new(p.FDCAN3, p.PF6, p.PF7, Irqs);
    can2.set_bitrate(500_000);
    can2.set_fd_data_bitrate(1_000_000, true);
    can2.set_tx_mode(can::config::TxBufferMode::Fifo);
    let can2 = can2.into_normal_mode();
    let (mut can_tx_2, can_rx_2, can_tx_event_2, can_cnt_2) = can2.split_with_control();

    let can_rx_stream_0 = create_can_rx_stream(can_rx_0, 0);
    let can_rx_stream_1 = create_can_rx_stream(can_rx_1, 1);
    let can_rx_stream_2 = create_can_rx_stream(can_rx_2, 2);

    let can_tx_event_stream_0 = create_can_tx_event_stream(can_tx_event_0, 0);
    let can_tx_event_stream_1 = create_can_tx_event_stream(can_tx_event_1, 1);
    let can_tx_event_stream_2 = create_can_tx_event_stream(can_tx_event_2, 2);

    let can_tx_channel = Channel::<NoopRawMutex, (FdFrame, u8, Option<u8>), 6>::new();

    let can_tx_fut = async {
        loop {
            let (frame, channel, echo_id) = can_tx_channel.receive().await;
            let tx_res = match channel {
                0 => can_tx_0.write_fd_with_marker(&frame, echo_id).await,
                1 => can_tx_1.write_fd_with_marker(&frame, echo_id).await,
                2 => can_tx_2.write_fd_with_marker(&frame, echo_id).await,
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

    static CAN_HANDLER: StaticCell<CanControlHandler> = StaticCell::new();
    let can_handler = CAN_HANDLER.init(CanControlHandler::new(can_cnt_0, can_cnt_1, can_cnt_2));

    info!("CAN Configured");

    // Create the driver, from the HAL.
    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();

    // let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = true;
    let driver = Driver::new_fs(
        p.USB_OTG_HS,
        Irqs,
        p.PA12,
        p.PA11,
        EP_OUT_BUFFER.init([0u8; 256]),
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

    static USB_STATE: StaticCell<State> = StaticCell::new();

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut DEVICE_DESC.init([0; 256])[..],
        &mut CONFIG_DESC.init([0; 256])[..],
        &mut BOS_DESC.init([0; 256])[..],
        &mut [], // no msos descriptors
        &mut CONTROL_BUF.init([0; 128])[..],
    );

    // Create classes on the builder.
    let class = GsCanClass::new(&mut builder, USB_STATE.init(State::new()), 3, can_handler);

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_task(usb)));

    let (mut usb_tx, usb_rx) = class.split();

    let usb_rx = pin!(stream::unfold(usb_rx, |mut usb_rx| async move {
        usb_rx.wait_connection().await;
        debug!("USB RX connected");
        let frame = usb_rx.read_frame().await;
        debug!("Read Frame from USB");
        match frame {
            Ok(frame) => return Some((Event::UsbRx(frame), usb_rx)),
            Err(error) => warn!("Invalid Frame: Error {:?}", error),
        };

        None
    }));

    let usb_tx_channel = Channel::<NoopRawMutex, HostFrame, 6>::new();
    let usb_tx_fut = async {
        loop {
            let frame = usb_tx_channel.receive().await;

            if let Some(_) = usb_tx.wait_connection().now_or_never() {
                debug!("USB TX connected");

                let usb_tx_res = usb_tx.write_frame(&frame).await;

                if usb_tx_res.is_err() {
                    warn!("Add USB error handling!");
                }
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

    static GS_HOST_FRAMES: StaticCell<[[Option<HostFrame>; 10]; 3]> = StaticCell::new();
    let host_frames =
        GS_HOST_FRAMES.init(array_init::array_init(|_| array_init::array_init(|_| None)));

    let green_led_channel = Channel::<NoopRawMutex, (), 1>::new();
    let yellow_led_channel = Channel::<NoopRawMutex, (), 1>::new();
    let red_led_channel = Channel::<NoopRawMutex, (), 1>::new();

    let main_handler = async {
        while let Some(event) = selectors.next().await {
            match event {
                Event::CanRx(frame, ts, channel) => {
                    debug!(
                        "CanRx | CAN Frame received. Channel: {}, Timestamp: {}, BRS: {}",
                        channel,
                        (ts.as_micros() as f64) / 1_000_000.0f64,
                        frame.header().bit_rate_switching()
                    );

                    let host_frame =
                        HostFrame::new_from(&frame, channel, u32::MAX, ts.as_micros() as u32);

                    usb_tx_channel.send(host_frame).await;

                    let _ = green_led_channel.try_send(());
                }
                Event::CanTx(echo_id, ts, channel) => {
                    debug!(
                        "CanTx | CAN Frame Transmitted. Channel: {}, Timestamp: {}, Echo ID {}",
                        channel,
                        (ts.as_micros() as f64) / 1_000_000.0f64,
                        echo_id
                    );

                    let _ = yellow_led_channel.try_send(());

                    if let Some(channel_host_frames) = host_frames.get_mut(channel as usize) {
                        if let Some(host_frame) = channel_host_frames.get_mut((echo_id) as usize) {
                            let frame = mem::take(host_frame);
                            if let Some(mut frame) = frame {
                                frame.set_timestamp(ts.as_micros() as u32);

                                usb_tx_channel.send(frame).await;
                            } else {
                                warn!("CanTx | Here should be a frame but isn't!");
                            }

                            continue;
                        }
                    }

                    warn!("Add error handling!");
                }
                Event::UsbRx(frame) => {
                    let can_frame: FdFrame = (&frame).into();

                    let channel = frame.get_channel() as usize;
                    let echo_id = frame.get_echo_id() as usize;

                    debug!(
                        "UsbRx | Host Frame received. Channel: {}, Echo ID: {}",
                        channel, echo_id
                    );

                    if let Some(channel_host_frames) = host_frames.get_mut(channel) {
                        if let Some(host_frame) = channel_host_frames.get(echo_id) {
                            if host_frame.is_none() {
                                channel_host_frames[echo_id] = Some(frame);
                            } else {
                                warn!("UsbRx | Echo ID already in buffer. That should not happen");
                            }
                        } else {
                            warn!("UsbRx | Echo ID out of bounds!");
                        }
                    }

                    can_tx_channel
                        .send((can_frame, channel as u8, Some(echo_id as u8)))
                        .await;
                }
                Event::CanErr(_channel) => {
                    let _ = red_led_channel.try_send(());
                }
            }
        }
    };

    let mut led_green = Output::new(p.PB0, Level::High, Speed::Low);
    let mut led_yellow = Output::new(p.PE1, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PB14, Level::Low, Speed::Low);

    let green_led_fut = async {
        let mut last_time = embassy_time::Instant::now();
        loop {
            if green_led_channel.receive().now_or_never().is_some() {
                if led_green.is_set_low() {
                    led_green.set_high();
                    Timer::after_millis(50).await;
                }

                led_green.set_low();
                Timer::after_millis(50).await;
                led_green.set_high();
                Timer::after_millis(50).await;
                last_time = embassy_time::Instant::now();
            } else {
                if (embassy_time::Instant::now() - last_time).as_millis() > 500 {
                    led_green.toggle();
                    last_time = embassy_time::Instant::now();
                } else {
                    Timer::after_millis(50).await;
                }
            }
        }
    };

    let yellow_led_fut = async {
        loop {
            let _ = yellow_led_channel.receive().await;
            led_yellow.set_high();
            Timer::after_millis(50).await;
            led_yellow.set_low();
        }
    };

    let red_led_fut = async {
        loop {
            let _ = red_led_channel.receive().await;
            led_red.set_high();
            Timer::after_millis(50).await;
            led_red.set_low();
        }
    };

    info!("Start handlers");
    join4(
        join3(green_led_fut, yellow_led_fut, red_led_fut),
        main_handler,
        can_tx_fut,
        usb_tx_fut,
    )
    .await;
}
