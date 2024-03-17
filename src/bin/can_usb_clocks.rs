#![no_std]
#![no_main]

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::usb_otg::{Driver, Instance};
use embassy_stm32::{bind_interrupts, can, peripherals, usb_otg, Config};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use futures::future::join3;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    OTG_HS => usb_otg::InterruptHandler<peripherals::USB_OTG_HS>;
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello can_usb_clocks!");

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
        config.rcc.fdcan_clock_source = FdCanClockSource::PLL1_Q;
    }
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
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

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

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    let mut can = can::FdcanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);

    // 250k bps
    can.set_bitrate(250_000);

    let can = can.into_internal_loopback_mode();
    // let mut can = can.into_normal_mode();

    info!("CAN Configured");

    let mut i = 0;
    let mut last_read_ts = embassy_time::Instant::now();

    let can_fut = async {
        let (mut tx, mut rx) = can.split();
        // With split
        loop {
            let frame = can::frame::ClassicFrame::new_extended(0x123456F, &[i; 8]).unwrap();
            info!("Writing frame");
            _ = tx.write(&frame).await;

            match rx.read().await {
                Ok((rx_frame, ts)) => {
                    let delta = (ts - last_read_ts).as_millis();
                    last_read_ts = ts;
                    info!(
                        "Rx: {:x} {:x} {:x} {:x} --- NEW {}",
                        rx_frame.data()[0],
                        rx_frame.data()[1],
                        rx_frame.data()[2],
                        rx_frame.data()[3],
                        delta,
                    )
                }
                Err(_err) => error!("Error in frame"),
            }

            Timer::after_millis(250).await;

            i = if i == 0xff { 0 } else { i + 1 };
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join3(usb_fut, echo_fut, can_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}
