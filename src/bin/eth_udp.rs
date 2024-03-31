#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Ipv4Cidr, Stack, StackResources};
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;
use embassy_stm32::rng::Rng;
use embassy_stm32::{bind_interrupts, eth, peripherals, rng, Config};
use embassy_time::Timer;
use embedded_io_async::Write;
use rand_core::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

type Device = Ethernet<'static, ETH, GenericSMI>;

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device>) -> ! {
    stack.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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
        config.rcc.mux.rngsel = mux::Rngsel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    // Generate random seed.
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut seed = [0; 8];
    rng.fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device = Ethernet::new(
        PACKETS.init(PacketQueue::<4, 4>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        GenericSMI::new(0),
        mac_addr,
    );

    // let config = embassy_net::Config::dhcpv4(Default::default());

    let ip_address = Ipv4Address::new(192, 168, 250, 61);
    let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
        address: Ipv4Cidr::new(ip_address, 24),
        dns_servers: heapless::Vec::new(),
        gateway: Some(ip_address),
    });

    // Init network stack
    static STACK: StaticCell<Stack<Device>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        device,
        config,
        RESOURCES.init(StackResources::<3>::new()),
        seed,
    ));

    // Launch network task
    unwrap!(spawner.spawn(net_task(&stack)));

    // Ensure DHCP configuration is up before trying connect
    stack.wait_config_up().await;

    if let Some(ip_v4) = stack.config_v4() {
        info!("IP config: {:?}", ip_v4.address);
    } else {
        warn!("No IP v4 config");
    }

    info!("Network task initialized");

    // Then we can use it!
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 4096];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 4096];

    loop {
        let mut socket = UdpSocket::new(
            stack,
            &mut rx_meta,
            &mut rx_buffer,
            &mut tx_meta,
            &mut tx_buffer,
        );

        socket.bind(0).unwrap();

        let remote_endpoint = (Ipv4Address::new(224, 4, 4, 4), 4444);

        info!("Starting loop");
        loop {
            let r = socket.send_to(b"Hello\n", remote_endpoint).await;
            if let Err(e) = r {
                info!("write error: {:?}", e);
                // break;
            }
            Timer::after_secs(1).await;
        }
    }
}
