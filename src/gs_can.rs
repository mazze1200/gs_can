//! CDC-ACM class implementation, aka Serial over USB.

use core::cell::{Cell, RefCell};
use core::future::poll_fn;
use core::mem::{self, MaybeUninit};
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use defmt::{debug, info, warn};
use embassy_sync::waitqueue::WakerRegistration;
use flagset::{flags, FlagSet};
use zerocopy_derive::{AsBytes, FromBytes};

use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

use zerocopy::{AsBytes, FromZeroes, Ref};
use zerocopy_derive::FromZeroes;

use zerocopy::byteorder::little_endian::{U32, U64};

use enumflags2::{bitflags, make_bitflags, BitFlags};

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_CLASS_GS_CAN: u8 = 0xff; // vendor specific

const GS_CAN_SUBCLASS_ACM: u8 = 0xff; // vendor specific
const GS_CAN_PROTOCOL_NONE: u8 = 0xff; // vendor specific

const CS_INTERFACE: u8 = 0x24;
const GS_CAN_TYPE_HEADER: u8 = 0x00;
const GS_CAN_TYPE_ACM: u8 = 0x02;
const GS_CAN_TYPE_UNION: u8 = 0x06;

const GS_USB_BREQ_HOST_FORMAT: u8 = 0;
const GS_USB_BREQ_BITTIMING: u8 = 1;
const GS_USB_BREQ_MODE: u8 = 2;
#[allow(unused)]
const GS_USB_BREQ_BERR: u8 = 3;
const GS_USB_BREQ_BT_CONST: u8 = 4;
const GS_USB_BREQ_DEVICE_CONFIG: u8 = 5;
const GS_USB_BREQ_TIMESTAMP: u8 = 6;
const GS_USB_BREQ_IDENTIFY: u8 = 7;
#[allow(unused)]
const GS_USB_BREQ_GET_USER_ID: u8 = 8;
#[allow(unused)]
const GS_USB_BREQ_QUIRK_CANTACT_PRO_DATA_BITTIMING: u8 = GS_USB_BREQ_GET_USER_ID;
#[allow(unused)]
const GS_USB_BREQ_SET_USER_ID: u8 = 9;
const GS_USB_BREQ_DATA_BITTIMING: u8 = 10;
const GS_USB_BREQ_BT_CONST_EXT: u8 = 11;
const GS_USB_BREQ_SET_TERMINATION: u8 = 12;
const GS_USB_BREQ_GET_TERMINATION: u8 = 13;
const GS_USB_BREQ_GET_STATE: u8 = 14;

const NUM_CAN_CHANNEL: u8 = 3;

/**
#define GS_CAN_MODE_NORMAL 0
#define GS_CAN_MODE_LISTEN_ONLY BIT(0)
#define GS_CAN_MODE_LOOP_BACK BIT(1)
#define GS_CAN_MODE_TRIPLE_SAMPLE BIT(2)
#define GS_CAN_MODE_ONE_SHOT BIT(3)
#define GS_CAN_MODE_HW_TIMESTAMP BIT(4)
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE BIT(7)
#define GS_CAN_MODE_FD BIT(8)
#define GS_CAN_MODE_BERR_REPORTING BIT(12)
 */

flags! {
    #[repr(u32)]
    enum GsDeviceModeFlag: u32 {
        // #define GS_CAN_MODE_NORMAL 0 /// this is not possible to outline
        GS_CAN_MODE_LISTEN_ONLY = 1<<0,
        GS_CAN_MODE_LOOP_BACK = 1<<1,
        GS_CAN_MODE_TRIPLE_SAMPLE = 1<<2,
        GS_CAN_MODE_ONE_SHOT = 1<<3,
        GS_CAN_MODE_HW_TIMESTAMP = 1<<4,
        GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE = 1<<7,
        GS_CAN_MODE_FD = 1<<8,
        GS_CAN_MODE_BERR_REPORTING = 1<<12
    }
}

struct GsDeviceModeFlags(FlagSet<GsDeviceModeFlag>);

impl GsDeviceModeFlags {
    fn new(flags: impl Into<FlagSet<GsDeviceModeFlag>>) -> GsDeviceModeFlags {
        GsDeviceModeFlags(flags.into())
    }
}

impl Into<U32> for GsDeviceModeFlags {
    fn into(self) -> U32 {
        self.0.bits().into()
    }
}

impl From<U32> for GsDeviceModeFlags {
    fn from(value: U32) -> Self {
        GsDeviceModeFlags(FlagSet::<GsDeviceModeFlag>::new_truncated(value.into()))
    }
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceMode {
    mode: U32,
    flags: U32,
}

impl GsDeviceMode {
    fn get_flags(&self) -> GsDeviceModeFlags {
        self.flags.into()
    }
    fn set_flags(&mut self, flags: GsDeviceModeFlags) {
        self.flags.set(flags.0.bits());
    }
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsTimestamp {
    timestamp: U32,
}
#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceState {
    state: U32,
    rxerr: U32,
    txerr: U32,
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceBittiming {
    prop_seg: U32,
    phase_seg1: U32,
    phase_seg2: U32,
    sjw: U32,
    brp: U32,
}
#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsIdentifyMode {
    mode: U32,
}
#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceTerminationState {
    state: U32,
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceConfig {
    reserved1: u8,
    reserved2: u8,
    reserved3: u8,
    icount: u8,
    sw_version: U32,
    hw_version: U32,
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsHostConfig {
    byte_order: U32,
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceBtConst {
    feature: U32,
    fclk_can: U32,
    tseg1_min: U32,
    tseg1_max: U32,
    tseg2_min: U32,
    tseg2_max: U32,
    sjw_max: U32,
    brp_min: U32,
    brp_max: U32,
    brp_inc: U32,
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
struct GsDeviceBtConstExtended {
    feature: U32,
    fclk_can: U32,
    tseg1_min: U32,
    tseg1_max: U32,
    tseg2_min: U32,
    tseg2_max: U32,
    sjw_max: U32,
    brp_min: U32,
    brp_max: U32,
    brp_inc: U32,

    dtseg1_min: U32,
    dtseg1_max: U32,
    dtseg2_min: U32,
    dtseg2_max: U32,
    dsjw_max: U32,
    dbrp_min: U32,
    dbrp_max: U32,
    dbrp_inc: U32,
}

/* Device specific constants */

/// Internal state for CDC-ACM
pub struct State<'a> {
    control: MaybeUninit<Control<'a>>,
    shared: ControlShared,
}

impl<'a> Default for State<'a> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'a> State<'a> {
    /// Create a new `State`.
    pub fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: ControlShared::default(),
        }
    }
}

#[derive(AsBytes, FromZeroes)]
#[repr(u8)]
enum GsHostFrameFlags {
    A,         // 0
    B = 5,     // 5
    C,         // 6
    D = 1 + 1, // 2
    E,         // 3
}

#[derive(AsBytes, FromZeroes)]
#[repr(C, packed)]
struct GsHostFrame {
    echo_id: U32,
    can_id: U32,
    can_dlc: u8,
    channel: u8,
    flags: GsHostFrameFlags,
    reserved: u8,
    data: [u8; 64],
    timestamp: U64,
}

/// Packet level implementation of a CDC-ACM serial port.
///
/// This class can be used directly and it has the least overhead due to directly reading and
/// writing USB packets with no intermediate buffers, but it will not act like a stream-like serial
/// port. The following constraints must be followed if you use this class directly:
///
/// - `read_packet` must be called with a buffer large enough to hold `max_packet_size` bytes.
/// - `write_packet` must not be called with a buffer larger than `max_packet_size` bytes.
/// - If you write a packet that is exactly `max_packet_size` bytes long, it won't be processed by the
///   host operating system until a subsequent shorter packet is sent. A zero-length packet (ZLP)
///   can be sent if there is no other data to send. This is because USB bulk transactions must be
///   terminated with a short packet, even if the bulk endpoint is used for stream-like data.
pub struct GsCanClass<'d, D: Driver<'d>> {
    _comm_ep: D::EndpointIn,
    read_ep: D::EndpointOut,
    write_ep: D::EndpointIn,
    control: &'d ControlShared,
}

struct Control<'a> {
    comm_if: InterfaceNumber,
    shared: &'a ControlShared,

    num_can_channels: u8,
}

/// Shared data between Control and CdcAcmClass
struct ControlShared {
    waker: RefCell<WakerRegistration>,
    changed: AtomicBool,
}

impl Default for ControlShared {
    fn default() -> Self {
        ControlShared {
            waker: RefCell::new(WakerRegistration::new()),
            changed: AtomicBool::new(false),
        }
    }
}

impl ControlShared {
    async fn changed(&self) {
        poll_fn(|cx| {
            if self.changed.load(Ordering::Relaxed) {
                self.changed.store(false, Ordering::Relaxed);
                Poll::Ready(())
            } else {
                self.waker.borrow_mut().register(cx.waker());
                Poll::Pending
            }
        })
        .await;
    }
}

impl<'a> Control<'a> {
    fn shared(&mut self) -> &'a ControlShared {
        self.shared
    }
}

impl<'d> Handler for Control<'d> {
    fn reset(&mut self) {
        let shared = self.shared();

        shared.changed.store(true, Ordering::Relaxed);
        shared.waker.borrow_mut().wake();
    }

    fn control_out(&mut self, req: control::Request, data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Vendor,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            warn!(
                "Received control_out ({:?}, {:?}, {:?})",
                req.request_type, req.recipient, req.index
            );
            return None;
        }

        match req.request {
            GS_USB_BREQ_HOST_FORMAT => {
                let data: Option<(Ref<_, GsHostConfig>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((host_config, _)) => {
                        info!(
                            "Received Host Config: {:#010x}",
                            host_config.byte_order.get()
                        );
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_HOST_FORMAT");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_MODE => {
                let data: Option<(Ref<_, GsDeviceMode>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((mode, _)) => {
                        let flags = mode.get_flags();
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_MODE");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_BITTIMING => {
                let data: Option<(Ref<_, GsDeviceBittiming>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((_bit_timing, _)) => Some(OutResponse::Accepted),
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_BITTIMING");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_DATA_BITTIMING => {
                let data: Option<(Ref<_, GsDeviceBittiming>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((_data_bit_timing, _)) => Some(OutResponse::Accepted),
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_DATA_BITTIMING");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_IDENTIFY => {
                let data: Option<(Ref<_, GsIdentifyMode>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((_identify_mode, _)) => Some(OutResponse::Accepted),
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_IDENTIFY");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_SET_TERMINATION => {
                let data: Option<(Ref<_, GsDeviceTerminationState>, _)> =
                    Ref::new_from_prefix(data);
                match data {
                    Some((_termination_state, _)) => Some(OutResponse::Accepted),
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_SET_TERMINATION");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            _ => {
                info!("Unknown control_out request: {}", req.request);
                None
            }
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if (req.request_type, req.recipient, req.index)
            != (
                RequestType::Vendor,
                Recipient::Interface,
                self.comm_if.0 as u16,
            )
        {
            debug!(
                "Received control_in ({:?}, {:?}, {:?})",
                req.request_type, req.recipient, req.index
            );
            return None;
        }

        match req.request {
            GS_USB_BREQ_DEVICE_CONFIG => {
                let data: Option<(Ref<_, GsDeviceConfig>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut device_config, _)) => {
                        device_config.reserved1 = 0;
                        device_config.reserved2 = 0;
                        device_config.reserved3 = 0;
                        device_config.icount = self.num_can_channels - 1; // The -1 is necessary since the original driver expects this. d'oh
                        device_config.hw_version.set(1);
                        device_config.hw_version.set(2);

                        Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_DEVICE_CONFIG");
                        Some(InResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_TIMESTAMP => {
                let data: Option<(Ref<_, GsTimestamp>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut timestamp, _)) => {
                        timestamp.timestamp.set(0);
                        Some(InResponse::Accepted(timestamp.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_TIMESTAMP");
                        Some(InResponse::Rejected)
                    }
                }
            }

            GS_USB_BREQ_GET_STATE => {
                let data: Option<(Ref<_, GsDeviceState>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut device_state, _)) => {
                        device_state.rxerr.set(0);
                        device_state.txerr.set(0);
                        device_state.state.set(0);
                        Some(InResponse::Accepted(device_state.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_GET_STATE");
                        Some(InResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_GET_TERMINATION => {
                let data: Option<(Ref<_, GsDeviceTerminationState>, _)> =
                    Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut terminaton_state, _)) => {
                        terminaton_state.state.set(0);
                        Some(InResponse::Accepted(terminaton_state.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_GET_TERMINATION");
                        Some(InResponse::Rejected)
                    }
                }
            }

            GS_USB_BREQ_BT_CONST => {
                let data: Option<(Ref<_, GsDeviceBtConst>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut bt_const, _)) => {
                        bt_const.brp_inc.set(0);
                        Some(InResponse::Accepted(bt_const.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_BT_CONST");
                        Some(InResponse::Rejected)
                    }
                }
            }
            GS_USB_BREQ_BT_CONST_EXT => {
                let data: Option<(Ref<_, GsDeviceBtConstExtended>, _)> =
                    Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut bt_const_ext, _)) => {
                        bt_const_ext.brp_inc.set(0);
                        Some(InResponse::Accepted(bt_const_ext.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_BT_CONST_EXT");
                        Some(InResponse::Rejected)
                    }
                }
            }

            _ => {
                info!("Unknown control_in request: {}", req.request);
                None
            }
        }
    }
}

impl<'d, D: Driver<'d>> GsCanClass<'d, D> {
    /// Creates a new CdcAcmClass with the provided UsbBus and `max_packet_size` in bytes. For
    /// full-speed devices, `max_packet_size` has to be one of 8, 16, 32 or 64.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        num_can_devices: u8,
    ) -> Self {
        assert!(builder.control_buf_len() >= 7);

        let mut func =
            builder.function(USB_CLASS_GS_CAN, GS_CAN_SUBCLASS_ACM, GS_CAN_PROTOCOL_NONE);

        // Control interface
        let mut iface = func.interface();
        let comm_if = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_CLASS_GS_CAN,
            GS_CAN_SUBCLASS_ACM,
            GS_CAN_PROTOCOL_NONE,
            None,
        );

        let comm_ep = alt.endpoint_interrupt_in(64, 255);

        let write_ep = alt.endpoint_bulk_in(32);
        let read_ep = alt.endpoint_bulk_out(32);

        drop(func);

        let control = state.control.write(Control {
            shared: &state.shared,
            comm_if,
            num_can_channels: num_can_devices,
        });
        builder.handler(control);

        let control_shared = &state.shared;

        GsCanClass {
            _comm_ep: comm_ep,
            read_ep,
            write_ep,
            control: control_shared,
        }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    /// Writes a single packet into the IN endpoint.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Reads a single packet from the OUT endpoint.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    /// Split the class into a sender and receiver.
    ///
    /// This allows concurrently sending and receiving packets from separate tasks.
    pub fn split(self) -> (Sender<'d, D>, Receiver<'d, D>) {
        (
            Sender {
                write_ep: self.write_ep,
                control: self.control,
            },
            Receiver {
                read_ep: self.read_ep,
                control: self.control,
            },
        )
    }

    /// Split the class into sender, receiver and control
    ///
    /// Allows concurrently sending and receiving packets whilst monitoring for
    /// control changes (dtr, rts)
    pub fn split_with_control(self) -> (Sender<'d, D>, Receiver<'d, D>, ControlChanged<'d>) {
        (
            Sender {
                write_ep: self.write_ep,
                control: self.control,
            },
            Receiver {
                read_ep: self.read_ep,
                control: self.control,
            },
            ControlChanged {
                control: self.control,
            },
        )
    }
}

/// CDC ACM Control status change monitor
///
/// You can obtain a `ControlChanged` with [`CdcAcmClass::split_with_control`]
pub struct ControlChanged<'d> {
    control: &'d ControlShared,
}

impl<'d> ControlChanged<'d> {
    /// Return a future for when the control settings change
    pub async fn control_changed(&self) {
        self.control.changed().await;
    }
}

/// CDC ACM class packet sender.
///
/// You can obtain a `Sender` with [`CdcAcmClass::split`]
pub struct Sender<'d, D: Driver<'d>> {
    write_ep: D::EndpointIn,
    control: &'d ControlShared,
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.write_ep.info().max_packet_size
    }

    /// Writes a single packet into the IN endpoint.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.write_ep.wait_enabled().await;
    }
}

/// CDC ACM class packet receiver.
///
/// You can obtain a `Receiver` with [`CdcAcmClass::split`]
pub struct Receiver<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
    control: &'d ControlShared,
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    /// Reads a single packet from the OUT endpoint.
    /// Must be called with a buffer large enough to hold max_packet_size bytes.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }
}
