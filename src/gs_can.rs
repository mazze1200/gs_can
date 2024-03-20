//! CDC-ACM class implementation, aka Serial over USB.

use core::array::from_mut;
use core::cell::RefCell;
use core::mem::{size_of, MaybeUninit};
use core::num::{NonZeroU16, NonZeroU8};
use core::ops::BitOrAssign;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, info, warn};
use embassy_stm32::can::frame;
use embassy_stm32::can::{
    config::{DataBitTiming, NominalBitTiming},
    frame::{FdFrame, Header},
};
use embassy_sync::waitqueue::WakerRegistration;

use embedded_can::{ExtendedId, Id, StandardId};
use flagset::{flags, FlagSet, InvalidBits};
use num_derive::{FromPrimitive, ToPrimitive};
use num_traits::{FromPrimitive, ToPrimitive};

use zerocopy_derive::{AsBytes, FromBytes};

use embassy_usb::control::{self, InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::driver::Endpoint;
use embassy_usb::driver::{Driver, EndpointError, EndpointIn, EndpointOut};

use embassy_usb::types::InterfaceNumber;
use embassy_usb::{Builder, Handler};

use zerocopy::{AsBytes, FromZeroes, Ref};
use zerocopy_derive::FromZeroes;

use zerocopy::byteorder::little_endian::U32;

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

    pub fn new() -> Self {
        GsDeviceBtConstExtended::new_zeroed()
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

// type CANID = U32;
#[derive(AsBytes, FromZeroes, FromBytes, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, packed)]
struct GsCanId(U32);

impl GsCanId {
    fn new(can_id: u32, extended: bool, rtr: bool, err: bool) -> Self {
        let mut id = if extended {
            U32::from(can_id & 0x1FFF_FFFF)
        } else {
            U32::from(can_id & 0x7FF)
        };
        if extended {
            id.bitor_assign(U32::new(0x8000_0000));
        }
        if rtr {
            id.bitor_assign(U32::new(0x4000_0000));
        }
        if err {
            id.bitor_assign(U32::new(0x2000_0000));
        }

        Self(id)
    }

    fn Id(&self) -> u32 {
        let id = self.0.get();
        if id & 0x8000_0000 == 0x8000_0000 {
            id & 0x1FFF_FFFF
        } else {
            id & 0x7FF
        }
    }

    fn extended(&self) -> bool {
        self.0.get() & 0x8000_0000 == 0x8000_0000
    }

    fn rtr(&self) -> bool {
        self.0.get() & 0x4000_0000 == 0x4000_0000
    }

    fn err(&self) -> bool {
        self.0.get() & 0x2000_0000 == 0x2000_0000
    }
}

#[derive(AsBytes, FromZeroes, FromBytes, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, packed)]
pub struct GsHostFrameClassic {
    pub echo_id: U32,
    pub can_id: GsCanId,
    pub can_dlc: u8,
    pub channel: u8,
    flags: u8,
    pub reserved: u8,
    pub data: [u8; 8],
}

#[derive(AsBytes, FromZeroes, FromBytes, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, packed)]
pub struct GsHostFrameClassicTs {
    pub echo_id: U32,
    pub can_id: GsCanId,
    pub can_dlc: u8,
    pub channel: u8,
    flags: u8,
    pub reserved: u8,
    pub data: [u8; 8],
    pub timestamp: U32,
}

#[derive(AsBytes, FromZeroes, FromBytes, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, packed)]
pub struct GsHostFrameFd {
    pub echo_id: U32,
    pub can_id: GsCanId,
    pub can_dlc: u8,
    pub channel: u8,
    flags: u8,
    pub reserved: u8,
    pub data: [u8; 64],
}

#[derive(AsBytes, FromZeroes, FromBytes, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, packed)]
pub struct GsHostFrameFdTs {
    pub echo_id: U32,
    pub can_id: GsCanId,
    pub can_dlc: u8,
    pub channel: u8,
    flags: u8,
    pub reserved: u8,
    pub data: [u8; 64],
    pub timestamp: U32,
}

trait GsHostFrame {
    fn get_flags(&self) -> Result<GsHostFrameFlags, InvalidBits>;
    fn set_flags(&mut self, flags: GsHostFrameFlags);
}

impl GsHostFrame for GsHostFrameClassic {
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
impl GsHostFrame for GsHostFrameClassicTs {
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
impl GsHostFrame for GsHostFrameFd {
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
impl GsHostFrame for GsHostFrameFdTs {
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

pub enum HostFrame {
    Classic(GsHostFrameClassic),
    ClassicTs(GsHostFrameClassicTs),
    Fd(GsHostFrameFd),
    FdTs(GsHostFrameFdTs),
}

impl HostFrame {
    pub fn new_from(frame: &FdFrame, channel: u8, echo_id: u32, timestamp: u32) -> Self {
        let header = frame.header();
        let id = match header.id() {
            Id::Standard(id) => GsCanId::new(id.as_raw() as u32, false, header.rtr(), false),
            Id::Extended(id) => GsCanId::new(id.as_raw(), true, header.rtr(), false),
        };

        if header.fdcan() {
            let mut hf = GsHostFrameFdTs::new_zeroed();
            hf.can_id = id;
            hf.can_dlc = header.len();
            hf.channel = channel;
            hf.echo_id.set(echo_id);
            hf.timestamp.set(timestamp);
            hf.data.copy_from_slice(frame.data());

            hf.set_flags(if header.bit_rate_switching() {
                GsHostFrameFlags(GsHostFrameFlag::GsCanFlagFd | GsHostFrameFlag::GsCanFlagBrs)
            } else {
                GsHostFrameFlags::new(GsHostFrameFlag::GsCanFlagFd)
            });

            HostFrame::FdTs(hf)
        } else {
            let mut hf = GsHostFrameClassicTs::new_zeroed();
            hf.can_id = id;
            hf.can_dlc = header.len();
            hf.channel = channel;
            hf.echo_id.set(echo_id);
            hf.timestamp.set(timestamp);
            hf.data.copy_from_slice(frame.data());

            HostFrame::ClassicTs(hf)
        }
    }

    pub fn get_channel(&self) -> u8 {
        match self {
            HostFrame::ClassicTs(frame) => frame.channel,
            HostFrame::Classic(frame) => frame.channel,
            HostFrame::Fd(frame) => frame.channel,
            HostFrame::FdTs(frame) => frame.channel,
        }
    }

    pub fn get_echo_id(&self) -> u32 {
        match self {
            HostFrame::ClassicTs(frame) => frame.echo_id.get(),
            HostFrame::Classic(frame) => frame.echo_id.get(),
            HostFrame::Fd(frame) => frame.echo_id.get(),
            HostFrame::FdTs(frame) => frame.echo_id.get(),
        }
    }
}

impl Into<FdFrame> for &HostFrame {
    fn into(self) -> FdFrame {
        match self {
            HostFrame::Classic(frame) => FdFrame::new(
                Header::new(
                    if frame.can_id.extended() {
                        embedded_can::Id::Extended(ExtendedId::new(frame.can_id.Id()).unwrap())
                    } else {
                        embedded_can::Id::Standard(
                            StandardId::new(frame.can_id.Id() as u16).unwrap(),
                        )
                    },
                    frame.can_dlc,
                    frame.can_id.rtr(),
                ),
                &frame.data[..],
            )
            .unwrap(),
            HostFrame::ClassicTs(frame) => FdFrame::new(
                Header::new(
                    if frame.can_id.extended() {
                        embedded_can::Id::Extended(ExtendedId::new(frame.can_id.Id()).unwrap())
                    } else {
                        embedded_can::Id::Standard(
                            StandardId::new(frame.can_id.Id() as u16).unwrap(),
                        )
                    },
                    frame.can_dlc,
                    frame.can_id.rtr(),
                ),
                &frame.data[..],
            )
            .unwrap(),
            HostFrame::Fd(frame) => FdFrame::new(
                Header::new_fd(
                    if frame.can_id.extended() {
                        embedded_can::Id::Extended(ExtendedId::new(frame.can_id.Id()).unwrap())
                    } else {
                        embedded_can::Id::Standard(
                            StandardId::new(frame.can_id.Id() as u16).unwrap(),
                        )
                    },
                    frame.can_dlc,
                    frame.can_id.rtr(),
                    frame
                        .get_flags()
                        .unwrap()
                        .0
                        .contains(GsHostFrameFlag::GsCanFlagBrs),
                ),
                &frame.data[..],
            )
            .unwrap(),
            HostFrame::FdTs(frame) => FdFrame::new(
                Header::new_fd(
                    if frame.can_id.extended() {
                        embedded_can::Id::Extended(ExtendedId::new(frame.can_id.Id()).unwrap())
                    } else {
                        embedded_can::Id::Standard(
                            StandardId::new(frame.can_id.Id() as u16).unwrap(),
                        )
                    },
                    frame.can_dlc,
                    frame.can_id.rtr(),
                    frame
                        .get_flags()
                        .unwrap()
                        .0
                        .contains(GsHostFrameFlag::GsCanFlagBrs),
                ),
                &frame.data[..],
            )
            .unwrap(),
        }
    }
}

// impl GsHostFrame {
//     pub fn new_from(frame: &FdFrame, channel: u8, echo_id: u32, timestamp: u32) -> Self {
//         let mut host_frame = GsHostFrame::new_zeroed();

//         let header = frame.header();
//         host_frame.can_dlc = header.len();
//         match header.id() {
//             embedded_can::Id::Extended(id) => {
//                 host_frame.can_id.set(id.as_raw() | 0x8000_0000);
//             }
//             embedded_can::Id::Standard(id) => host_frame.can_id.set(id.as_raw() as u32),
//         };
//         host_frame.set_flags(GsHostFrameFlags::new(GsHostFrameFlag::GsCanFlagFd));
//         host_frame.channel = channel;
//         host_frame.echo_id.set(echo_id);
//         host_frame.timestamp.set(timestamp);

//         host_frame
//     }
// }

// impl Into<FdFrame> for &GsHostFrame {
//     fn into(self) -> FdFrame {
//         let mask_11 = 0b111_1111_1111u32;
//         let mask_29 = 0b1_1111_1111_1111_1111_1111_1111_1111u32;
//         let id: u32 = self.can_id.get();
//         let id: embedded_can::Id = if (id & 0x8000_0000) == 0x8000_0000 {
//             embedded_can::Id::Extended(embedded_can::ExtendedId::new(id & mask_29).unwrap())
//         } else {
//             embedded_can::Id::Standard(
//                 embedded_can::StandardId::new((id & mask_11) as u16).unwrap(),
//             )
//         };

//         FdFrame::new(Header::new(id, self.can_dlc, false), &self.data[..]).unwrap()
//     }
// }

pub struct GsCanClass<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
    write_ep: D::EndpointIn,
    _dummy_read_ep: D::EndpointOut,
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

impl<'a> Control<'a> {
    fn shared(&mut self) -> &'a ControlShared {
        self.shared
    }
}

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

                info!("GsUsbBreqHostFormat");

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

                info!("GsUsbBreqMode");
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

                info!("GsUsbBreqBittiming");
                match data {
                    Some((bit_timing, _)) => {
                        info!("Set bit timing for CAN {}", req.value);
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

                info!("GsUsbBreqDataBittiming");
                match data {
                    Some((data_bit_timing, _)) => {
                        info!("Set data_bittiming for CAN {}", req.value);
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

                info!("GsUsbBreqIdentify");
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

                info!("GsUsbBreqSetTermination");
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
                info!("GsUsbBreqDeviceConfig");

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
                info!("GsUsbBreqTimestamp");

                match data {
                    Some((mut timestamp, _)) => {
                        let ts = self.can_handlers.get_timestamp();

                        info!("Send Timestamp {}", ts.as_micros() as u32);
                        timestamp.timestamp.set(ts.as_micros() as u32);
                        // timestamp.timestamp.set(0);

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
                info!("GsUsbBreqGetState");

                match data {
                    Some((_device_state, _)) => {
                        info!("Not implemented. This is dependant on the feature GsCanFeatureGetState");
                        Some(InResponse::Rejected)
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
                info!("GsUsbBreqGetTermination");

                match data {
                    Some((_terminaton_state, _)) => {
                        info!("Not implemented. This is dependant on the feature GsCanFeatureGetState");
                        Some(InResponse::Rejected)
                        // Some(InResponse::Accepted(buf))
                    }
                    None => {
                        info!("unaligned buffer for: GS_USB_BREQ_GET_TERMINATION");
                        Some(InResponse::Rejected)
                    }
                }
            }
            Some(GsUsbRequestType::GsUsbBreqBtConst) => {
                let data: Option<(Ref<_, GsDeviceBtConst>, _)> = Ref::new_from_prefix(&mut *buf);
                info!("GsUsbBreqBtConst");

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
                info!("GsUsbBreqBtConstExt");

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

        let write_ep = alt.endpoint_bulk_in(64);

        // This dummy EP is necessary, since the GS_CAN driver expects the out EP 2 but the first out EP would be 1
        let _dummy_read_ep = alt.endpoint_bulk_out(64);
        let read_ep = alt.endpoint_bulk_out(64);

        drop(func);

        let control = state.control.write(Control {
            shared: &state.shared,
            comm_if,
            can_handlers,
            num_can_channels,
        });
        builder.handler(control);

        GsCanClass {
            read_ep,
            write_ep,
            _dummy_read_ep,
        }
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
            },
        )
    }
}

/// CDC ACM class packet sender.
///
/// You can obtain a `Sender` with [`CdcAcmClass::split`]
pub struct Sender<'d, D: Driver<'d>> {
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Writes a single frame into the IN endpoint.
    pub async fn write_frame(&mut self, frame: &HostFrame) -> Result<(), EndpointError> {
        // self.write_ep.write(data).await

        info!("Sending Frame");

        // let buf = frame.as_bytes();

        // self.write_ep.write(&buf[0..31]).await?;
        // self.write_ep.write(&buf[32..63]).await?;
        // self.write_ep.write(&buf[64..]).await?;

        Ok(())
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
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    // pub async fn read_frame(&mut self) -> Result<GsHostFrame, EndpointError> {
    //     let mut buf = [0u8; 96];

    //     info!("Receiving Frame");

    //     self.read_ep.read(&mut buf[0..31]).await?;
    //     info!("Received first buffer");

    //     self.read_ep.read(&mut buf[32..63]).await?;
    //     info!("Received first buffer");

    //     self.read_ep.read(&mut buf[64..]).await?;

    //     let re: Option<(Ref<_, GsHostFrame>, _)> = Ref::new_from_prefix(&mut buf[..]);

    //     if let Some((frame, _buffer)) = re {
    //         return Ok(frame.clone());
    //     }

    //     Err(EndpointError::BufferOverflow)
    // }

    pub async fn read_frame(&mut self) -> Result<HostFrame, EndpointError> {
        let mut buf = [0u8; 128];

        info!("Receiving Frame");

        let mut total_bytes = 0usize;

        loop {
            let read_bytes = self.read_ep.read(&mut buf[total_bytes..]).await?;
            total_bytes += read_bytes;
            if read_bytes < self.read_ep.info().max_packet_size as usize {
                break;
            }
        }

        info!(
            "Received buffer len: {} | {:?}",
            total_bytes,
            buf[..total_bytes]
        );

        // let re: Option<(Ref<_, GsHostFrame>, _)> = Ref::new_from_prefix(&mut buf[..]);

        // if let Some((frame, _)) = re {
        //     let clone = frame.clone();
        //     info!("GsHostFrame: echo_id {}", clone.echo_id.get());
        //     return Ok(clone);
        // }

        Err(EndpointError::BufferOverflow)
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }
}
