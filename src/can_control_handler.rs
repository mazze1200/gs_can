use defmt::debug;
use embassy_stm32::can::{FdCanConfiguration, FdcanControl};
use embassy_stm32::peripherals::{FDCAN1, FDCAN2, FDCAN3};

use crate::gs_can::{self, GsCanHandlers, GsDeviceBittiming, GsDeviceBtConstFeature};
use embassy_stm32::can;

use defmt_rtt as _;


pub struct CanControlHandler {
    can_cnt_0: FdcanControl<FDCAN1>,
    can_cnt_1: FdcanControl<FDCAN2>,
    can_cnt_2: FdcanControl<FDCAN3>,
}

impl CanControlHandler {
    pub fn new(can_cnt_0: FdcanControl<FDCAN1>,  can_cnt_1: FdcanControl<FDCAN2>, can_cnt_2: FdcanControl<FDCAN3>,) -> Self {
        CanControlHandler{
            can_cnt_0,
            can_cnt_1,
            can_cnt_2
        }
    }
}

impl GsCanHandlers for CanControlHandler {
    fn get_timestamp(&self) -> embassy_time::Instant {
        embassy_time::Instant::now()
    }

    fn set_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming) {
        if let Some(bit_timing) = Into::<Option<can::config::NominalBitTiming>>::into(timing) {
            let channel: Option<&mut dyn can::FdCanConfiguration> = match channel {
                0 => Some(&mut self.can_cnt_0),
                1 => Some(&mut self.can_cnt_1),
                2 => Some(&mut self.can_cnt_2),
                _ => None,
            };

            if let Some(channel) = channel {
                let current_timing = channel.get_config().get_nominal_bit_timing();
                if current_timing != bit_timing {
                    debug!("New Bit Timing {}", bit_timing);
                    channel.into_config_mode();
                    channel.set_bitrate(bit_timing);
                    channel.start(can::FdcanOperatingMode::NormalOperationMode);
                }
            }
        }
    }

    fn set_data_bittiming(&mut self, channel: u16, timing: &GsDeviceBittiming) {
        if let Some(data_bit_timing) = Into::<Option<can::config::DataBitTiming>>::into(timing) {
            let channel: Option<&mut dyn can::FdCanConfiguration> = match channel {
                0 => Some(&mut self.can_cnt_0),
                1 => Some(&mut self.can_cnt_1),
                2 => Some(&mut self.can_cnt_2),
                _ => None,
            };

            if let Some(channel) = channel {
                let current_timing = channel.get_config().get_data_bit_timing();
                if current_timing != data_bit_timing {
                    debug!("New Data Bit Timing {}", data_bit_timing);
                    channel.into_config_mode();
                    channel.set_fd_data_bitrate(data_bit_timing);
                    channel.start(can::FdcanOperatingMode::NormalOperationMode);
                }
            }
        }
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
                    | GsDeviceBtConstFeature::GsCanFeatureIdentify,
            );

            timing.fclk_can.set(frequency.0);
            timing.tseg1_min.set(1);
            timing.tseg1_max.set(256);
            timing.tseg2_min.set(1);
            timing.tseg2_max.set(128);
            timing.sjw_max.set(128);
            timing.brp_min.set(1);
            timing.brp_max.set(512);
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
            timing.tseg1_max.set(256);
            timing.tseg2_min.set(1);
            timing.tseg2_max.set(128);
            timing.sjw_max.set(128);
            timing.brp_min.set(1);
            timing.brp_max.set(512);
            timing.brp_inc.set(1);

            timing.dtseg1_min.set(1);
            timing.dtseg1_max.set(32);
            timing.dtseg2_min.set(1);
            timing.dtseg2_max.set(16);
            timing.dsjw_max.set(16);
            timing.dbrp_min.set(1);
            timing.dbrp_max.set(32);
            timing.dbrp_inc.set(1);
        }
    }
}
