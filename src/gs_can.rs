//! CDC-ACM class implementation, aka Serial over USB.

use core::borrow::BorrowMut;
use core::cell::{Cell, RefCell};
use core::clone;
use core::future::poll_fn;
use core::mem::{self, size_of, size_of_val, MaybeUninit};
use core::num::{NonZeroU16, NonZeroU8};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use defmt::{debug, info, warn};
use embassy_stm32::can::config::{DataBitTiming, NominalBitTiming};
use embassy_stm32::can::frame::{ClassicData, ClassicFrame, FdData, FdFrame, Header};
use embassy_stm32::can::CanFrame;
use embassy_sync::waitqueue::WakerRegistration;
use embedded_hal::can::{ExtendedId, Id, StandardId};
use flagset::{flags, FlagSet, InvalidBits};
use futures::{Future, FutureExt, Stream, TryFutureExt};
use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::{FromPrimitive, ToPrimitive};
use zerocopy_derive::{AsBytes, FromBytes};

use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

use zerocopy::{AsBytes, FromZeroes, Ref};
use zerocopy_derive::FromZeroes;

use zerocopy::byteorder::little_endian::{U32};

use enumflags2::{bitflags, make_bitflags, BitFlags};

use futures::prelude::*;


// extern  crate embedded_can;

/// This should be used as `device_class` when building the `UsbDevice`.
const USB_CLASS_GS_CAN: u8 = 0xff; // vendor specific
const GS_CAN_SUBCLASS_ACM: u8 = 0xff; // vendor specific
const GS_CAN_PROTOCOL_NONE: u8 = 0xff; // vendor specific

// const NUM_CAN_CHANNEL: u8 = 3;

// const GS_USB_BREQ_HOST_FORMAT: u8 = 0;
// const GS_USB_BREQ_BITTIMING: u8 = 1;
// const GS_USB_BREQ_MODE: u8 = 2;
// #[allow(unused)]
// const GS_USB_BREQ_BERR: u8 = 3;
// const GS_USB_BREQ_BT_CONST: u8 = 4;
// const GS_USB_BREQ_DEVICE_CONFIG: u8 = 5;
// const GS_USB_BREQ_TIMESTAMP: u8 = 6;
// const GS_USB_BREQ_IDENTIFY: u8 = 7;
// #[allow(unused)]
// const GS_USB_BREQ_GET_USER_ID: u8 = 8;
// #[allow(unused)]
// const GS_USB_BREQ_QUIRK_CANTACT_PRO_DATA_BITTIMING: u8 = GS_USB_BREQ_GET_USER_ID;
// #[allow(unused)]
// const GS_USB_BREQ_SET_USER_ID: u8 = 9;
// const GS_USB_BREQ_DATA_BITTIMING: u8 = 10;
// const GS_USB_BREQ_BT_CONST_EXT: u8 = 11;
// const GS_USB_BREQ_SET_TERMINATION: u8 = 12;
// const GS_USB_BREQ_GET_TERMINATION: u8 = 13;
// const GS_USB_BREQ_GET_STATE: u8 = 14;

#[derive(FromPrimitive, ToPrimitive, Debug)]
enum GsUsbRequestType {
    GsUsbBreqHostFormat = 0,
    GsUsbBreqBittiming = 1,
    GsUsbBreqMode = 2,
    GsUsbBreqBerr = 3,
    GsUsbBreqBtConst = 4,
    GsUsbBreqDeviceConfig = 5,
    GsUsbBreqTimestamp = 6,
    GsUsbBreqIdentify = 7,
    GsUsbBreqGetUserId = 8,
    GsUsbBreqSetUserId = 9,
    GsUsbBreqDataBittiming = 10,
    GsUsbBreqBtConstExt = 11,
    GsUsbBreqSetTermination = 12,
    GsUsbBreqGetTermination = 13,
    GsUsbBreqGetState = 14,
}

/*
    /* reset a channel. turns it off */
    GS_CAN_MODE_RESET = 0,
    /* starts a channel */
    GS_CAN_MODE_START


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
        // #define GS_CAN_MODE_NORMAL = 0 /// this is not possible to outline
        GsCanModeListenOnly = 1<<0,
        GsCanModeLoopBack = 1<<1,
        GsCanModeTripleSample = 1<<2,
        GsCanModeOneShot = 1<<3,
        GsCanModeHwTimestamp = 1<<4,
        GsCanModePadPktsToMaxPktSize = 1<<7,
        GsCanModeFd = 1<<8,
        GsCanModeBerrReporting = 1<<12
    }
}

#[derive(FromPrimitive, ToPrimitive)]
enum GsDeviceModeMode {
    // #define GS_CAN_MODE_NORMAL 0 /// this is not possible to outline
    GsCanModeReset = 0,
    GsCanModeStart = 1,
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
    fn get_mode(&self) -> Option<GsDeviceModeMode> {
        FromPrimitive::from_u32(self.mode.into())
    }

    fn set_mode(&mut self, mode: GsDeviceModeMode) {
        self.mode.set(mode.to_u32().unwrap());
    }

    fn get_flags(&self) -> Result<GsDeviceModeFlags, InvalidBits> {
        // self.flags.into()
        match FlagSet::<GsDeviceModeFlag>::new(self.flags.into()) {
            Ok(flags_set) => Ok(GsDeviceModeFlags::new(flags_set)),
            Err(invalid) => Err(invalid),
        }
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
pub struct GsDeviceBittiming {
    prop_seg: U32,
    phase_seg1: U32,
    phase_seg2: U32,
    sjw: U32,
    brp: U32,
}

impl Into<Option<NominalBitTiming>> for &GsDeviceBittiming {
    fn into(self) -> Option<NominalBitTiming> {
        Some(NominalBitTiming {
            prescaler: NonZeroU16::new(self.brp.get() as u16)?,
            seg1: NonZeroU8::new((self.prop_seg.get() + self.phase_seg1.get()) as u8)?,
            seg2: NonZeroU8::new(self.phase_seg2.get() as u8)?,
            sync_jump_width: NonZeroU8::new(self.sjw.get() as u8)?,
        })
    }
}

impl Into<Option<DataBitTiming>> for &GsDeviceBittiming {
    fn into(self) -> Option<DataBitTiming> {
        Some(DataBitTiming {
            transceiver_delay_compensation: false,
            prescaler: NonZeroU16::new(self.brp.get() as u16)?,
            seg1: NonZeroU8::new((self.prop_seg.get() + self.phase_seg1.get()) as u8)?,
            seg2: NonZeroU8::new(self.phase_seg2.get() as u8)?,
            sync_jump_width: NonZeroU8::new(self.sjw.get() as u8)?,
        })
    }
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

flags! {
    #[repr(u32)]
    pub enum GsDeviceBtConstFeature: u32 {
        GsCanFeatureListenOnly =1<<0,
        GsCanFeatureLoopBack =1<<1,
        GsCanFeatureTripleSample =1<<2,
        GsCanFeatureOneShot =1<<3,
        GsCanFeatureHwTimestamp =1<<4,
        GsCanFeatureIdentify =1<<5,
        GsCanFeatureFd =1<<8,
        GsCanFeatureBtConstExt =1<<10,
        GsCanFeatureTermination =1<<11,
        GsCanFeatureBerrReporting =1<<12,
        GsCanFeatureGetState =1<<13,
    }
}

pub struct GsDeviceBtConstFeatures(FlagSet<GsDeviceBtConstFeature>);

impl GsDeviceBtConstFeatures {
    pub fn new(flags: impl Into<FlagSet<GsDeviceBtConstFeature>>) -> GsDeviceBtConstFeatures {
        GsDeviceBtConstFeatures(flags.into())
    }
}

impl Into<u32> for GsDeviceBtConstFeatures {
    fn into(self) -> u32 {
        self.0.bits().into()
    }
}

impl From<u32> for GsDeviceBtConstFeatures {
    fn from(value: u32) -> Self {
        GsDeviceBtConstFeatures(FlagSet::<GsDeviceBtConstFeature>::new_truncated(value))
    }
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
pub struct GsDeviceBtConst {
    feature: U32,
    pub fclk_can: U32,
    pub tseg1_min: U32,
    pub tseg1_max: U32,
    pub tseg2_min: U32,
    pub tseg2_max: U32,
    pub sjw_max: U32,
    pub brp_min: U32,
    pub brp_max: U32,
    pub brp_inc: U32,
}

impl GsDeviceBtConst {
    pub fn get_features(&self) -> Result<GsDeviceBtConstFeatures, InvalidBits> {
        match FlagSet::<GsDeviceBtConstFeature>::new(self.feature.into()) {
            Ok(flags_set) => Ok(GsDeviceBtConstFeatures::new(flags_set)),
            Err(invalid) => Err(invalid),
        }
    }
    pub fn set_features(&mut self, flags: FlagSet<GsDeviceBtConstFeature>) {
        self.feature.set(flags.bits());
    }
}

#[derive(FromZeroes, FromBytes, AsBytes)]
#[repr(C, packed)]
pub struct GsDeviceBtConstExtended {
    feature: U32,
    pub fclk_can: U32,
    pub tseg1_min: U32,
    pub tseg1_max: U32,
    pub tseg2_min: U32,
    pub tseg2_max: U32,
    pub sjw_max: U32,
    pub brp_min: U32,
    pub brp_max: U32,
    pub brp_inc: U32,

    pub dtseg1_min: U32,
    pub dtseg1_max: U32,
    pub dtseg2_min: U32,
    pub dtseg2_max: U32,
    pub dsjw_max: U32,
    pub dbrp_min: U32,
    pub dbrp_max: U32,
    pub dbrp_inc: U32,
}

impl GsDeviceBtConstExtended {
    pub fn get_features(&self) -> Result<GsDeviceBtConstFeatures, InvalidBits> {
        match FlagSet::<GsDeviceBtConstFeature>::new(self.feature.into()) {
            Ok(flags_set) => Ok(GsDeviceBtConstFeatures::new(flags_set)),
            Err(invalid) => Err(invalid),
        }
    }
    pub fn set_features(&mut self, flags: FlagSet<GsDeviceBtConstFeature>) {
        self.feature.set(flags.bits());
    }
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

flags! {
    #[repr(u8)]
    enum GsHostFrameFlag: u8 {
        GsCanFlagOverflow = 1<<0,
        GsCanFlagFd = 1<<1,
        GsCanFlagBrs = 1<<2,
        GsCanFlagEsi = 1<<3
    }
}

struct GsHostFrameFlags(FlagSet<GsHostFrameFlag>);

impl GsHostFrameFlags {
    fn new(flags: impl Into<FlagSet<GsHostFrameFlag>>) -> GsHostFrameFlags {
        GsHostFrameFlags(flags.into())
    }
}

impl Into<u8> for GsHostFrameFlags {
    fn into(self) -> u8 {
        self.0.bits().into()
    }
}

impl From<u8> for GsHostFrameFlags {
    fn from(value: u8) -> Self {
        GsHostFrameFlags(FlagSet::<GsHostFrameFlag>::new_truncated(value))
    }
}

#[derive(AsBytes, FromZeroes, FromBytes, Clone)]
#[repr(C, packed)]
pub struct GsHostFrame {
    pub echo_id: U32,
    pub can_id: U32,
    pub can_dlc: u8,
    pub channel: u8,
    pub flags: u8,
    pub reserved: u8,
    pub data: [u8; 64],
    pub timestamp: U32,
}

impl GsHostFrame {

    // fn new() -> Self{
    //     let buf= [0u8;mem::size_of::<GsHostFrame>()];
    //     zerocopy::transmute!(buf)
    // }

    fn get_flags(&self) -> Result<GsHostFrameFlags, InvalidBits> {
        // self.flags.into()
        match FlagSet::<GsHostFrameFlag>::new(self.flags.into()) {
            Ok(flags_set) => Ok(GsHostFrameFlags::new(flags_set)),
            Err(invalid) => Err(invalid),
        }
    }
    fn set_flags(&mut self, flags: GsHostFrameFlags) {
        self.flags = flags.0.bits();
    }
}

impl Into<CanFrame> for &GsHostFrame{
    fn into(self) -> CanFrame {
        let id: embedded_can::Id = if (self.can_id.get() & 0x8000_0000) == 0x8000_0000{
            embedded_can::Id::Extended( embedded_can::ExtendedId::new(self.can_id.get()).unwrap())
        }else{ 
            embedded_can::Id::Standard(embedded_can::StandardId::new(self.can_id.get() as u16).unwrap())
        };

        if self.get_flags().unwrap().0.contains(GsHostFrameFlag::GsCanFlagFd) {
            CanFrame::FD(FdFrame::new(Header::new( id,self.can_dlc , false), FdData::new(&self.data[..]).unwrap()))
        }else{
            CanFrame::Classic(ClassicFrame::new(Header::new( id,self.can_dlc , false), ClassicData::new(&self.data[..]).unwrap()))
        }
    }
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
    can_handlers: &'a mut dyn GsCanHandlers,
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

// pub struct GsCanHandlers {
//     pub get_timestamp: fn() -> embassy_time::Instant,
//     pub set_bittiming: fn(channel: u16, timing: Ref<&[u8], GsDeviceBittiming>),
//     pub set_data_bittiming: fn(channel: u16, timing: Ref<&[u8], GsDeviceBittiming>),
//     pub get_bittiming: fn(channel: u16, timing: Ref<&mut [u8], GsDeviceBtConst>),
//     pub get_bittiming_extended: fn(channel: u16, timing: Ref<&mut [u8], GsDeviceBtConstExtended>),
// }

pub trait GsCanHandlers {
    fn get_timestamp(&self) -> embassy_time::Instant;
    fn set_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming);
    fn set_data_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming);
    fn get_bittiming(&self, channel: u16, timing: &mut GsDeviceBtConst);
    fn get_bittiming_extended(&self, channel: u16, timing: &mut GsDeviceBtConstExtended);
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

        match FromPrimitive::from_u8(req.request) {
            Some(GsUsbRequestType::GsUsbBreqHostFormat) => {
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
            Some(GsUsbRequestType::GsUsbBreqMode) => {
                let data: Option<(Ref<_, GsDeviceMode>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((mode, _)) => {
                        match mode.get_mode() {
                            Some(GsDeviceModeMode::GsCanModeReset) => info!("todo!"),
                            Some(GsDeviceModeMode::GsCanModeStart) => match mode.get_flags() {
                                Ok(flags) => info!("todo!"),
                                Err(_) => info!("todo!"),
                            },
                            None => info!("todo!"),
                        }

                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_MODE");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqBittiming) => {
                let data: Option<(Ref<_, GsDeviceBittiming>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((bit_timing, _)) => {
                        self.can_handlers.set_bittiming(req.value, &bit_timing);
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_BITTIMING");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqDataBittiming) => {
                let data: Option<(Ref<_, GsDeviceBittiming>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((data_bit_timing, _)) => {
                        self.can_handlers
                            .set_data_bittiming(req.value, &data_bit_timing);
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_DATA_BITTIMING");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqIdentify) => {
                let data: Option<(Ref<_, GsIdentifyMode>, _)> = Ref::new_from_prefix(data);
                match data {
                    Some((_identify_mode, _)) => {
                        info!("todo");
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_IDENTIFY");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqSetTermination) => {
                let data: Option<(Ref<_, GsDeviceTerminationState>, _)> =
                    Ref::new_from_prefix(data);
                match data {
                    Some((_termination_state, _)) => {
                        info!("todo");
                        Some(OutResponse::Accepted)
                    }
                    None => {
                        warn!("unaligned buffer for: GS_USB_BREQ_SET_TERMINATION");
                        Some(OutResponse::Rejected)
                    }
                }
            }

            Some(_) => {
                warn!("Unhandled control_out request: {}", req.request);
                None
            }
            _ => {
                warn!("Unknown control_out request: {}", req.request);
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

        // match req.request {
        match FromPrimitive::from_u8(req.request) {
            Some(GsUsbRequestType::GsUsbBreqDeviceConfig) => {
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
            Some(GsUsbRequestType::GsUsbBreqTimestamp) => {
                let data: Option<(Ref<_, GsTimestamp>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut timestamp, _)) => {
                        let ts = self.can_handlers.get_timestamp();

                        timestamp.timestamp.set(ts.as_micros() as u32);
                        // Some(InResponse::Accepted(timestamp.into_ref().as_bytes()))

                        Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_TIMESTAMP");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqGetState) => {
                let data: Option<(Ref<_, GsDeviceState>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((_device_state, _)) => {
                        todo!("is dependant on the feature GsCanFeatureGetState");
                        // device_state.rxerr.set(0);
                        // device_state.txerr.set(0);
                        // device_state.state.set(0);
                        // Some(InResponse::Accepted(device_state.into_ref().as_bytes()))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_GET_STATE");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqGetTermination) => {
                let data: Option<(Ref<_, GsDeviceTerminationState>, _)> =
                    Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((_terminaton_state, _)) => {
                        todo!("is dependant on the feature GsCanFeatureTermination");
                        Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_GET_TERMINATION");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqBtConst) => {
                let data: Option<(Ref<_, GsDeviceBtConst>, _)> = Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut bt_const, _)) => {
                        self.can_handlers.get_bittiming(req.value, &mut bt_const);
                        Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_BT_CONST");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqBtConstExt) => {
                let data: Option<(Ref<_, GsDeviceBtConstExtended>, _)> =
                    Ref::new_from_prefix(&mut *buf);

                match data {
                    Some((mut bt_const_ext, _)) => {
                        self.can_handlers
                            .get_bittiming_extended(req.value, &mut bt_const_ext);
                        Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_BT_CONST_EXT");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(_) => {
                warn!("Unhandled control_in request: {}", req.request);
                None
            }
            _ => {
                warn!("Unknown control_in request: {}", req.request);
                None
            }
        }
    }
}

impl<'d, D> GsCanClass<'d, D>
where
    D: Driver<'d>,
{
    /// Creates a new CdcAcmClass with the provided UsbBus and `max_packet_size` in bytes. For
    /// full-speed devices, `max_packet_size` has to be one of 8, 16, 32 or 64.
    pub fn new(
        builder: &mut Builder<'d, D>,
        state: &'d mut State<'d>,
        num_can_channels: u8,
        can_handlers: &'d mut dyn GsCanHandlers,
    ) -> Self {
        assert!(num_can_channels <= 3);
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
            can_handlers,
            num_can_channels,
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

    /// Split the class into a sender and receiver.
    ///
    /// This allows concurrently sending and receiving packets from separate tasks.
    pub fn split(self) -> (Sender<'d, D>, Receiver<'d, D>) {
        (
            Sender {
                write_ep: self.write_ep,
            },
            Receiver {
                read_ep: self.read_ep,
                // control: self.control,
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
            },
            Receiver {
                read_ep: self.read_ep,
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
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.write_ep.info().max_packet_size
    }

    /// Writes a single frame into the IN endpoint.
    pub async fn write_frame(&mut self, frame: &GsHostFrame) -> Result<(), EndpointError> {
        // self.write_ep.write(data).await

        let buf = frame.as_bytes();

        self.write_ep.write(&buf[0..31]).await?;
        self.write_ep.write(&buf[32..63]).await?;
        self.write_ep.write(&buf[64..]).await?;

        Ok(())
    }
}

/// CDC ACM class packet receiver.
///
/// You can obtain a `Receiver` with [`CdcAcmClass::split`]
pub struct Receiver<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    pub async fn read_frame(&mut self) -> Result<GsHostFrame, EndpointError> {
        let mut buf = [0u8; 96];

        assert_eq!(self.read_ep.read(&mut buf[0..31]).await?, 32);
        assert_eq!(self.read_ep.read(&mut buf[32..63]).await?, 32);
        assert_eq!(self.read_ep.read(&mut buf[64..]).await?, 20);

        let re: Option<(Ref<_, GsHostFrame>, _)> = Ref::new_from_prefix(&mut buf[..]);

        if let Some((frame, _buffer)) = re {
            return Ok(frame.clone());
        }

        Err(EndpointError::BufferOverflow)
    }
}
