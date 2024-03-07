use core::cell::{RefCell, RefMut};
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::task::Poll;

use embassy_hal_internal::{into_ref, PeripheralRef};
pub use fdcan;
pub use fdcan::frame::{FrameFormat, RxFrameInfo, TxFrameHeader};
pub use fdcan::id::{ExtendedId, Id, StandardId};
use fdcan::message_ram::RegisterBlock;
use fdcan::LastErrorCode;
pub use fdcan::{config, filter};
use futures::Stream;

use crate::gpio::sealed::AFType;
use crate::interrupt::typelevel::Interrupt;
use crate::rcc::RccPeripheral;
use crate::{interrupt, peripherals, Peripheral};

// as far as I can tell, embedded-hal/can doesn't have any fdcan frame support
pub struct RxFrame {
    pub header: RxFrameInfo,
    pub data: Data,
}

pub struct TxFrame {
    pub header: TxFrameHeader,
    pub data: Data,
}

impl TxFrame {
    pub fn new(header: TxFrameHeader, data: &[u8]) -> Option<Self> {
        if data.len() < header.len as usize {
            return None;
        }

        let Some(data) = Data::new(data) else { return None };

        Some(TxFrame { header, data })
    }

    fn from_preserved(header: TxFrameHeader, data32: &[u32]) -> Option<Self> {
        //let data = unsafe { core::mem::transmute(*data32) };
        let mut data = [0u8; 64];

        for i in 0..data32.len() {
            data[4 * i..][..4].copy_from_slice(&data32[i].to_le_bytes());
        }

        let Some(data) = Data::new(&data) else { return None };

        Some(TxFrame { header, data })
    }

    pub fn data(&self) -> &[u8] {
        &self.data.bytes[..(self.header.len as usize)]
    }
}

impl RxFrame {
    pub(crate) fn new(header: RxFrameInfo, data: &[u8]) -> Self {
        let data = Data::new(&data).unwrap_or_else(|| Data::empty());

        RxFrame { header, data }
    }

    pub fn data(&self) -> &[u8] {
        &self.data.bytes[..(self.header.len as usize)]
    }
}

/// Payload of a (FD)CAN data frame.
///
/// Contains 0 to 64 Bytes of data.
#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub(crate) bytes: [u8; 64],
}

impl Data {
    /// Creates a data payload from a raw byte slice.
    ///
    /// Returns `None` if `data` is more than 64 bytes (which is the maximum) or
    /// cannot be represented with an FDCAN DLC.
    pub fn new(data: &[u8]) -> Option<Self> {
        if !Data::is_valid_len(data.len()) {
            return None;
        }

        let mut bytes = [0; 64];
        bytes[..data.len()].copy_from_slice(data);

        Some(Self { bytes })
    }

    pub fn raw(&self) -> &[u8] {
        &self.bytes
    }

    /// Checks if the length can be encoded in FDCAN DLC field.
    pub const fn is_valid_len(len: usize) -> bool {
        match len {
            0..=8 => true,
            12 => true,
            16 => true,
            20 => true,
            24 => true,
            32 => true,
            48 => true,
            64 => true,
            _ => false,
        }
    }

    /// Creates an empty data payload containing 0 bytes.
    #[inline]
    pub const fn empty() -> Self {
        Self { bytes: [0; 64] }
    }
}

/// Interrupt handler.
pub struct IT0InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

// We use IT0 for everything currently
impl<T: Instance> interrupt::typelevel::Handler<T::IT0Interrupt> for IT0InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // can't use T::regs().ir().read().tc()
        // metapac has wrong bit position for `TC` flag for H7 parts

        let regs = T::REGISTERS;
        let ir = (*regs).ir.read();
        info!("it0 interrupt: {:b}", ir.bits());

        if ir.tc().bit_is_set() {
            (*regs).ir.write(|w| w.tc().set_bit());
            T::state().tx_waker.wake();
            info!("tc: set tx_waker");
        }

        if ir.tcf().bit_is_set() {
            (*regs).ir.write(|w| w.tcf().set_bit());
            T::state().tx_waker.wake();
            info!("tef");
        }
        if ir.ped().bit_is_set() {
            (*regs).ir.write(|w| w.ped().set_bit());
            //T::state().tx_waker.wake();
            info!("ped");
        }

        if ir.rf0n().bit_is_set() {
            (*regs).ir.write(|w| w.rf0n().set_bit());
            T::state().rx0_waker.wake();
            info!("rfn0: set rx0_waker");
            //Fdcan::<T>::receive_fifo(RxFifo::Fifo0);
        }
        if ir.rf1n().bit_is_set() {
            (*regs).ir.write(|w| w.rf1n().set_bit());
            T::state().rx1_waker.wake();
            info!("rfn1: set rx1_waker");
            //Fdcan::<T>::receive_fifo(RxFifo::Fifo1);
        }

        if ir.pea().bit_is_set() {
            (*regs).ir.write(|w| w.pea().set_bit());
            let psr = T::regs().psr().read();
            if let Ok(err) = LastErrorCode::try_from(psr.lec()) {
                if let Some(bus_error) = BusError::try_from(err) {
                    let _ = T::state().err_queue.try_send(bus_error);
                }
            }
            info!("pea");
        }

        // if false {
        //     let err = { T::regs().psr().read() };
        //     if err.bo() {
        //         return Some(BusError::BusOff);
        //     } else if err.ep() {
        //         return Some(BusError::BusPassive);
        //     } else if err.ew() {
        //         return Some(BusError::BusWarning);
        //     } else if let Ok(err) = LastErrorCode::try_from(err.lec()) {
        //         return BusError::try_from(err);
        //     }

        //     let _ = T::state().err_queue.try_send(BusError::Acknowledge);
        // }
    }
}

pub struct IT1InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::IT1Interrupt> for IT1InterruptHandler<T> {
    unsafe fn on_interrupt() {
        info!("it1 interrupt");
    }
}

/*  TODO: handle error
impl<T: Instance> interrupt::typelevel::Handler<T::SCEInterrupt> for SceInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // info!("sce irq");
        let msr = T::regs().msr();
        let msr_val = msr.read();

        if msr_val.erri() {
            msr.modify(|v| v.set_erri(true));
            T::state().err_waker.wake();
        }
    }
}
 */

#[derive(Debug)]
pub enum BusError {
    Stuff,
    Form,
    Acknowledge,
    BitRecessive,
    BitDominant,
    Crc,
    Software,
    BusOff,
    BusPassive,
    BusWarning,
}

impl BusError {
    fn try_from(lec: LastErrorCode) -> Option<BusError> {
        match lec {
            LastErrorCode::AckError => Some(BusError::Acknowledge),
            LastErrorCode::Bit0Error => Some(BusError::BitRecessive), // TODO: verify
            LastErrorCode::Bit1Error => Some(BusError::BitDominant),  // TODO: verify
            LastErrorCode::CRCError => Some(BusError::Crc),
            LastErrorCode::FormError => Some(BusError::Form),
            LastErrorCode::StuffError => Some(BusError::Stuff),
            _ => None,
        }
    }
}

pub trait FdcanOperatingMode {}
impl FdcanOperatingMode for fdcan::PoweredDownMode {}
impl FdcanOperatingMode for fdcan::ConfigMode {}
impl FdcanOperatingMode for fdcan::InternalLoopbackMode {}
impl FdcanOperatingMode for fdcan::ExternalLoopbackMode {}
impl FdcanOperatingMode for fdcan::NormalOperationMode {}
impl FdcanOperatingMode for fdcan::RestrictedOperationMode {}
impl FdcanOperatingMode for fdcan::BusMonitoringMode {}
impl FdcanOperatingMode for fdcan::TestMode {}

/*
pub enum FdcanInstanceMode<T: fdcan::Instance> {
    PoweredDownMode(fdcan::FdCan<T, fdcan::PoweredDownMode>),
    ConfigMode(fdcan::FdCan<T, fdcan::ConfigMode>),
    InternalLoopbackMode(fdcan::FdCan<T, fdcan::InternalLoopbackMode>),
    ExternalLoopbackMode(fdcan::FdCan<T, fdcan::ExternalLoopbackMode>),
    NormalOperationMode(fdcan::FdCan<T, fdcan::NormalOperationMode>),
    RestrictedOperationMode(fdcan::FdCan<T, fdcan::RestrictedOperationMode>),
    BusMonitoringMode(fdcan::FdCan<T, fdcan::BusMonitoringMode>),
    TestMode(fdcan::FdCan<T, fdcan::TestMode>),
}
 */

pub struct Fdcan<'d, T: Instance, M: FdcanOperatingMode> {
    pub can: RefCell<fdcan::FdCan<FdcanInstance<'d, T>, M>>,
}

impl<'d, T: Instance> Fdcan<'d, T, fdcan::ConfigMode> {
    /// Creates a new Fdcan instance, keeping the peripheral in sleep mode.
    /// You must call [Fdcan::enable_non_blocking] to use the peripheral.
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        _irqs: impl interrupt::typelevel::Binding<T::IT0Interrupt, IT0InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::IT1Interrupt, IT1InterruptHandler<T>>
            + 'd,
    ) -> Fdcan<'d, T, fdcan::ConfigMode> {
        into_ref!(peri, rx, tx);

        rx.set_as_af(rx.af_num(), AFType::Input);
        tx.set_as_af(tx.af_num(), AFType::OutputPushPull);

        T::enable_and_reset();
        //   T::reset();
        info!("can peripheral running at {}", T::frequency());

        // crate::peripherals::FDCAN2

        //let can = fdcan::FdCan::builder(FdcanInstance(peri)).leave_disabled();
        let mut can = fdcan::FdCan::new(FdcanInstance(peri)).into_config_mode();

        unsafe {
            #[cfg(feature = "fdcan_h7")]
            {
                T::configure_msg_ram();
            }
            T::IT0Interrupt::unpend();
            T::IT0Interrupt::enable();

            T::IT1Interrupt::unpend();
            T::IT1Interrupt::enable();

            // this isn't really documented in the reference manual
            // but corresponding txbtie bit has to be set for the TC (TxComplete) interrupt to fire
            (*(T::REGISTERS)).txbtie.write(|w| w.bits(0xffffffff));
        }

        can.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        can.enable_interrupt(fdcan::interrupt::Interrupt::TxComplete);
        //can.enable_interrupt(fdcan::interrupt::Interrupt::ProtErrArbritation);
        //can.enable_interrupt(fdcan::interrupt::Interrupt::ProtErrData);
        can.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);
        can.enable_interrupt_line(fdcan::interrupt::InterruptLine::_1, true);

        let can_ref_cell = RefCell::new(can);
        Self { can: can_ref_cell }
    }

    /*
    pub fn into_config_mode(mut self) -> Fdcan<'d, T, fdcan::ConfigMode> {
        Fdcan { can: RefCell::new(self.can.into_inner().into_config_mode()) }
    }
     */
}

/*
impl<'d, T: Instance> Fdcan<'d, T, fdcan::ConfigMode> {
    pub fn into_normal_mode(mut self) -> Fdcan<'d, T, fdcan::NormalOperationMode> {
        Fdcan { can: RefCell::new(self.can.into_inner().into_normal()) }
    }
}
 */

macro_rules! impl_transition {
    ($from_mode:ident, $to_mode:ident, $name:ident, $func: ident) => {
        impl<'d, T: Instance> Fdcan<'d, T, fdcan::$from_mode> {
            pub fn $name(self) -> Fdcan<'d, T, fdcan::$to_mode> {
                Fdcan {
                    can: RefCell::new(self.can.into_inner().$func()),
                }
            }
        }
    };
}

impl_transition!(PoweredDownMode, ConfigMode, into_config_mode, into_config_mode);
impl_transition!(InternalLoopbackMode, ConfigMode, into_config_mode, into_config_mode);

impl_transition!(ConfigMode, NormalOperationMode, into_normal_mode, into_normal);
impl_transition!(
    ConfigMode,
    ExternalLoopbackMode,
    into_external_loopback_mode,
    into_external_loopback
);
impl_transition!(
    ConfigMode,
    InternalLoopbackMode,
    into_internal_loopback_mode,
    into_internal_loopback
);

impl<'d, T: Instance, M: FdcanOperatingMode> Fdcan<'d, T, M> {
    pub fn as_mut(&self) -> RefMut<'_, fdcan::FdCan<FdcanInstance<'d, T>, M>> {
        self.can.borrow_mut()
    }
}

impl<'d, T: Instance, M: FdcanOperatingMode> Fdcan<'d, T, M>
where
    M: fdcan::Transmit,
    M: fdcan::Receive,
{
    /// Queues the message to be sent but exerts backpressure.  If a lower-priority
    /// frame is dropped from the mailbox, it is returned.  If no lower-priority frames
    /// can be replaced, this call asynchronously waits for a frame to be successfully
    /// transmitted, then tries again.
    // pub async fn write(&mut self, frame: &TxFrame) -> Option<TxFrame> {
    //     poll_fn(|cx| {
    //         T::state().tx_waker.register(cx.waker());
    //         if let Ok(dropped) =
    //             self.can
    //                 .borrow_mut()
    //                 .transmit_preserve(frame.header, &frame.data.bytes, &mut |_, hdr, data32| {
    //                     TxFrame::from_preserved(hdr, data32)
    //                 })
    //         {
    //             return Poll::Ready(dropped.flatten());
    //         }

    //         // Couldn't replace any lower priority frames.  Need to wait for some mailboxes
    //         // to clear.
    //         debug!("fdcan: tx full, pending");
    //         Poll::Pending
    //     })
    //     .await
    // }

    /*
        // can't implement this right now because metapac's TRP definition is wrong
        pub async fn flush(&self, mb: fdcan::Mailbox) {
            poll_fn(|cx| {
                T::state().tx_waker.register(cx.waker());

                let idx: u8 = mb.into();
                let idx: u32 = 1u32 << (idx as u32);
                if !(T::regs().txbrp().read().trp(idx) != 0) {
                    return Poll::Ready(());
                }

                Poll::Pending
            })
            .await;
        }
    */

    /// Returns the next received message frame
    // pub async fn read(&mut self) -> Result<RxFrame, BusError> {
    //     poll_fn(|cx| {
    //         T::state().err_waker.register(cx.waker());
    //         T::state().rx_waker.register(cx.waker());

    //         // TODO: handle fifo0 AND fifo1
    //         let mut buffer: [u8; 64] = [0; 64];
    //         if let Ok(rx) = self.can.borrow_mut().receive0(&mut buffer) {
    //             // rx: fdcan::ReceiveOverrun<RxFrameInfo>
    //             // TODO: report overrun?
    //             //  for now we just drop it
    //             let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
    //             return Poll::Ready(Ok(frame));
    //         } else if let Some(err) = self.curr_error() {
    //             // TODO: this is probably wrong
    //             return Poll::Ready(Err(err));
    //         }

    //         Poll::Pending
    //     })
    //     .await
    // }

    fn curr_error(&self) -> Option<BusError> {
        let err = { T::regs().psr().read() };
        if err.bo() {
            return Some(BusError::BusOff);
        } else if err.ep() {
            return Some(BusError::BusPassive);
        } else if err.ew() {
            return Some(BusError::BusWarning);
        } else if let Ok(err) = LastErrorCode::try_from(err.lec()) {
            return BusError::try_from(err);
        }
        None
    }

    pub fn split<'c>(
        &'c self,
    ) -> (
        FdcanTx<'c, 'd, T, M>,
        FdcanRx<'c, 'd, T, M, RxFifo0>,
        FdcanRx<'c, 'd, T, M, RxFifo1>,
    ) {
        (
            FdcanTx { can: &self.can },
            FdcanRx {
                can: &self.can,
                _marker: PhantomData,
            },
            FdcanRx {
                can: &self.can,
                _marker: PhantomData,
            },
        )
    }
}

pub struct FdcanTx<'c, 'd, T: Instance, M: fdcan::Transmit> {
    can: &'c RefCell<fdcan::FdCan<FdcanInstance<'d, T>, M>>,
}

impl<'c, 'd, T: Instance, M: fdcan::Transmit> FdcanTx<'c, 'd, T, M> {
    /// Queues the message to be sent but exerts backpressure.  If a lower-priority
    /// frame is dropped from the mailbox, it is returned.  If no lower-priority frames
    /// can be replaced, this call asynchronously waits for a frame to be successfully
    /// transmitted, then tries again.
    pub async fn write(&mut self, frame: &TxFrame) -> Option<TxFrame> {
        poll_fn(|cx| {
            T::state().tx_waker.register(cx.waker());
            if let Ok(dropped) =
                self.can
                    .borrow_mut()
                    .transmit_preserve(frame.header, &frame.data.bytes, &mut |_, hdr, data32| {
                        TxFrame::from_preserved(hdr, data32)
                    })
            {
                return Poll::Ready(dropped.flatten());
            }

            // Couldn't replace any lower priority frames.  Need to wait for some mailboxes
            // to clear.
            debug!("fdcan: tx full, pending");
            Poll::Pending
        })
        .await
    }

    /*
    pub async fn flush(&self, mb: bxcan::Mailbox) {
        poll_fn(|cx| {
            T::state().tx_waker.register(cx.waker());
            if T::regs().tsr().read().tme(mb.index()) {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }*/
}

pub struct RxFifo0;
pub struct RxFifo1;

pub trait RxFifo {}
impl RxFifo for RxFifo0 {}
impl RxFifo for RxFifo1 {}

// #[allow(dead_code)]
// pub struct FdcanErr<'c, 'd, T: Instance> {
//     can: &'c RefCell<FdcanInstance<'d, T>>,
// }

// impl<'c, 'd, T: Instance> Stream for FdcanErr<'c, 'd, T> {
//     type Item = ();

//     fn poll_next(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Option<Self::Item>> {
//         // T::state().err_waker.register(cx.waker());
//         // T::state().rx0_waker.register(cx.waker());
//         // let _ = T::state().err_queue.try_send(LastErrorCode);

//         // TODO: handle fifo0 AND fifo1
//         // let mut buffer: [u8; 64] = [0; 64];
//         // if let Ok(rx) = self.can.borrow_mut().receive0(&mut buffer) {
//         //     let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
//         //     return Poll::Ready(Some(()));
//         // }

//         Poll::Pending
//     }
// }

#[allow(dead_code)]
pub struct FdcanTxEvent<'c, 'd, T: Instance> {
    can: &'c RefCell<FdcanInstance<'d, T>>,
}

impl<'c, 'd, T: Instance> Stream for FdcanTxEvent<'c, 'd, T> {
    type Item = ();

    fn poll_next(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Option<Self::Item>> {
        T::state().tx_event_waker.register(cx.waker());

        //self.can.borrow_mut()

        // T::state().err_waker.register(cx.waker());
        // T::state().rx0_waker.register(cx.waker());
        // let _ = T::state().err_queue.try_send(LastErrorCode);

        // TODO: handle fifo0 AND fifo1
        // let mut buffer: [u8; 64] = [0; 64];
        // if let Ok(rx) = self.can.borrow_mut().receive0(&mut buffer) {
        //     let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
        //     return Poll::Ready(Some(()));
        // }

        Poll::Pending
    }
}

#[allow(dead_code)]
pub struct FdcanRx<'c, 'd, T: Instance, M: fdcan::Receive, F: RxFifo> {
    can: &'c RefCell<fdcan::FdCan<FdcanInstance<'d, T>, M>>,
    _marker: PhantomData<F>,
}

impl<'c, 'd, T: Instance, M: fdcan::Receive, F: RxFifo> FdcanRx<'c, 'd, T, M, F> {
    fn curr_error(&self) -> Option<BusError> {
        let err = { T::regs().psr().read() };
        if err.bo() {
            return Some(BusError::BusOff);
        } else if err.ep() {
            return Some(BusError::BusPassive);
        } else if err.ew() {
            return Some(BusError::BusWarning);
        } else if let Ok(err) = LastErrorCode::try_from(err.lec()) {
            return BusError::try_from(err);
        }
        None
    }
}

impl<'c, 'd, T: Instance, M: fdcan::Receive> Stream for FdcanRx<'c, 'd, T, M, RxFifo0> {
    type Item = RxFrame;

    fn poll_next(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Option<Self::Item>> {
        // T::state().err_waker.register(cx.waker());
        T::state().rx0_waker.register(cx.waker());

        // TODO: handle fifo0 AND fifo1
        let mut buffer: [u8; 64] = [0; 64];
        if let Ok(rx) = self.can.borrow_mut().receive0(&mut buffer) {
            // rx: fdcan::ReceiveOverrun<RxFrameInfo>
            // TODO: report overrun?
            //  for now we just drop it
            let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
            return Poll::Ready(Some(frame));
        }
        //  else if let Some(err) = self.curr_error() {
        //     // TODO: this is probably wrong
        //     return Poll::Ready(Some(Err(err)));
        // }

        Poll::Pending
    }
}

impl<'c, 'd, T: Instance, M: fdcan::Receive> Stream for FdcanRx<'c, 'd, T, M, RxFifo1> {
    type Item = RxFrame;

    fn poll_next(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<Option<Self::Item>> {
        // T::state().err_waker.register(cx.waker());
        T::state().rx1_waker.register(cx.waker());

        // TODO: handle fifo0 AND fifo1
        let mut buffer: [u8; 64] = [0; 64];
        if let Ok(rx) = self.can.borrow_mut().receive1(&mut buffer) {
            // rx: fdcan::ReceiveOverrun<RxFrameInfo>
            // TODO: report overrun?
            //  for now we just drop it
            let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
            return Poll::Ready(Some(frame));
        }
        // else if let Some(err) = self.curr_error() {
        //     // TODO: this is probably wrong
        //     return Poll::Ready(Some(Err(err)));
        // }

        Poll::Pending
    }
}

// impl<'c, 'd, T: Instance, M: fdcan::Receive> FdcanRx<'c, 'd, T, M> {
//     /// Returns the next received message frame
//     pub async fn read(&mut self) -> Result<RxFrame, BusError> {
//         poll_fn(|cx| {
//             T::state().err_waker.register(cx.waker());
//             T::state().rx_waker.register(cx.waker());

//             // TODO: handle fifo0 AND fifo1
//             let mut buffer: [u8; 64] = [0; 64];
//             if let Ok(rx) = self.can.borrow_mut().receive0(&mut buffer) {
//                 // rx: fdcan::ReceiveOverrun<RxFrameInfo>
//                 // TODO: report overrun?
//                 //  for now we just drop it
//                 let frame: RxFrame = RxFrame::new(rx.unwrap(), &buffer);
//                 return Poll::Ready(Ok(frame));
//             } else if let Some(err) = self.curr_error() {
//                 // TODO: this is probably wrong
//                 return Poll::Ready(Err(err));
//             }

//             Poll::Pending
//         })
//         .await
//     }

//     fn curr_error(&self) -> Option<BusError> {
//         let err = { T::regs().psr().read() };
//         if err.bo() {
//             return Some(BusError::BusOff);
//         } else if err.ep() {
//             return Some(BusError::BusPassive);
//         } else if err.ew() {
//             return Some(BusError::BusWarning);
//         } else if let Ok(err) = LastErrorCode::try_from(err.lec()) {
//             return BusError::try_from(err);
//         }
//         None
//     }
// }

// TODO: impl Drop prevents us from changing modes without dropping the old one.
//  there is a solution here but I need to think about it more
/*
impl<'d, T: Instance, M: FdcanOperatingMode> Drop for Fdcan<'d, T, M> {
    fn drop(&mut self) {
        // Cannot call `free()` because it moves the instance.
        // Manually reset the peripheral.
        T::regs().cccr().write(|w| w.set_init(true));
        T::disable();
    }
}
 */

impl<'d, T: Instance, M: FdcanOperatingMode> Deref for Fdcan<'d, T, M> {
    type Target = RefCell<fdcan::FdCan<FdcanInstance<'d, T>, M>>;

    fn deref(&self) -> &Self::Target {
        &self.can
    }
}

impl<'d, T: Instance, M: FdcanOperatingMode> DerefMut for Fdcan<'d, T, M> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.can
    }
}

pub(crate) mod sealed {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;
    use embassy_sync::waitqueue::AtomicWaker;
    use fdcan::LastErrorCode;

    use super::BusError;

    pub struct State {
        pub tx_waker: AtomicWaker,
        pub tx_event_waker: AtomicWaker,
        pub rx0_waker: AtomicWaker,
        pub rx1_waker: AtomicWaker,
        pub err_queue: Channel<CriticalSectionRawMutex, BusError, 32>,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                tx_waker: AtomicWaker::new(),
                tx_event_waker: AtomicWaker::new(),
                rx0_waker: AtomicWaker::new(),
                rx1_waker: AtomicWaker::new(),
                err_queue: Channel::new(),
            }
        }
    }

    pub trait Instance {
        const REGISTERS: *mut fdcan::RegisterBlock;
        const MSG_RAM: *mut fdcan::message_ram::RegisterBlock;

        fn regs() -> &'static crate::pac::can::Fdcan;
        fn state() -> &'static State;
        //  unsafe fn configure_msg_ram();
        /*
        fn msg_ram(&self) -> &RegisterBlock;
        fn msg_ram_mut(&mut self) -> &mut RegisterBlock;
         */
    }
}

pub trait IT0Instance {
    type IT0Interrupt: crate::interrupt::typelevel::Interrupt;
}

pub trait IT1Instance {
    type IT1Interrupt: crate::interrupt::typelevel::Interrupt;
}

pub trait InterruptableInstance: IT0Instance + IT1Instance {}
pub trait Instance: sealed::Instance + RccPeripheral + InterruptableInstance + 'static {}

pub struct FdcanInstance<'a, T>(PeripheralRef<'a, T>);

/*
unsafe impl<'d, T: Instance> fdcan::message_ram::Instance for FdcanInstance<'d, T> {
    const MSG_RAM: *mut RegisterBlock = T::MSG_RAM;
}
*/
/*
unsafe impl<'d, T: Instance> fdcan::message_ram::Instance for FdcanInstance<'d, T> where T: fdcan::message_ram::Instance {
    const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = ;
}

unsafe impl<'d, T> fdcan::message_ram::Instance for FdcanInstance<'d, T> {
    const MSG_RAM: *mut RegisterBlock = (); // TODO ?
}


 */

unsafe impl<'d, T: Instance> fdcan::message_ram::Instance for FdcanInstance<'d, T> {
    const MSG_RAM: *mut RegisterBlock = T::MSG_RAM;
}

unsafe impl<'d, T: Instance> fdcan::Instance for FdcanInstance<'d, T>
where
    FdcanInstance<'d, T>: fdcan::message_ram::Instance,
{
    const REGISTERS: *mut fdcan::RegisterBlock = T::REGISTERS;
}

// This macro taken from stm32h7xx-hal and adapted here
/// Configure Message RAM layout on H7 to match the fixed sized used on G4
///
/// These are protected bits, write access is only possible when bit CCE and bit
/// INIT for FDCAN_CCCR are set to 1

macro_rules! impl_fdcan {
    ($inst:ident, $msg_ram_addr:literal, $msg_ram_offset:literal) => {
        impl sealed::Instance for peripherals::$inst {
            const REGISTERS: *mut fdcan::RegisterBlock = crate::pac::$inst.as_ptr() as *mut _;
            const MSG_RAM: *mut fdcan::message_ram::RegisterBlock = (($msg_ram_addr+$msg_ram_offset) as *mut _);

            fn regs() -> &'static crate::pac::can::Fdcan {
                &crate::pac::$inst
            }

            fn state() -> &'static sealed::State {
                static STATE: sealed::State = sealed::State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {}

        foreach_interrupt!(
            ($inst,can,FDCAN,IT0,$irq:ident) => {
                impl IT0Instance for peripherals::$inst {
                    type IT0Interrupt = crate::interrupt::typelevel::$irq;
                }
            };
            ($inst,can,FDCAN,IT1,$irq:ident) => {
                impl IT1Instance for peripherals::$inst {
                    type IT1Interrupt = crate::interrupt::typelevel::$irq;
                }
            };
        );

        impl InterruptableInstance for peripherals::$inst {}
    }
}

// see https://github.com/stm32-rs/stm32g4xx-hal/blob/master/src/can.rs
foreach_peripheral!(
    (can, FDCAN1) => { impl_fdcan!(FDCAN1, 0x4000_a400, 0); };
    (can, FDCAN2) => { impl_fdcan!(FDCAN2, 0x4000_a750, 0); };
    (can, FDCAN3) => { impl_fdcan!(FDCAN3, 0x4000_aaa0, 0); };
);

// hack stolen from similar situation in DAC
// the metapac lists the RCC registers under FDCAN1 so this causes a conflict with FDCAN2
// H7 uses single bit for both FDCAN1 and FDCAN2, this is a hack until a proper fix is implemented
// NOTE: disabling either FDCAN will affect the other!

impl crate::rcc::sealed::RccPeripheral for peripherals::FDCAN2 {
    fn frequency() -> crate::time::Hertz {
        peripherals::FDCAN1::frequency()
    }

    fn enable_and_reset_with_cs(cs: critical_section::CriticalSection) {
        todo!("implement a singleton for all CAN instances since they share this functionality and thus only needs to be done once");
    }

    fn disable_with_cs(cs: critical_section::CriticalSection) {
        todo!("implement a singleton for all CAN instances since they share this functionality and thus only needs to be done once");
    }
}

impl crate::rcc::RccPeripheral for peripherals::FDCAN2 {}

impl crate::rcc::sealed::RccPeripheral for peripherals::FDCAN3 {
    fn frequency() -> crate::time::Hertz {
        peripherals::FDCAN1::frequency()
    }

    fn enable_and_reset_with_cs(cs: critical_section::CriticalSection) {
        todo!("implement a singleton for all CAN instances since they share this functionality and thus only needs to be done once");
    }

    fn disable_with_cs(cs: critical_section::CriticalSection) {
        todo!("implement a singleton for all CAN instances since they share this functionality and thus only needs to be done once");
    }
}

impl crate::rcc::RccPeripheral for peripherals::FDCAN3 {}

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);

/*  /// copy from _generated.rs file
impl crate::rcc::sealed::RccPeripheral for peripherals::FDCAN1 {
    fn frequency() -> crate::time::Hertz {
        unsafe { crate::rcc::get_freqs().pclk1 }
    }
    fn enable_and_reset_with_cs(_cs: critical_section::CriticalSection) {
        #[cfg(feature = "low-power")]
        unsafe {
            crate::rcc::REFCOUNT_STOP2 += 1
        };
        crate::pac::RCC.apb1enr1().modify(|w| w.set_fdcanen(true));
        crate::pac::RCC.apb1rstr1().modify(|w| w.set_fdcanrst(true));
        crate::pac::RCC.apb1rstr1().modify(|w| w.set_fdcanrst(false));
    }
    fn disable_with_cs(_cs: critical_section::CriticalSection) {
        crate::pac::RCC.apb1enr1().modify(|w| w.set_fdcanen(false));
        #[cfg(feature = "low-power")]
        unsafe {
            crate::rcc::REFCOUNT_STOP2 -= 1
        };
    }
}
impl crate::rcc::RccPeripheral for peripherals::FDCAN1 {}
 */
