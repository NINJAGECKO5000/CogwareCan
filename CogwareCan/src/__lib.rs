#![no_std]
#![no_main]
#![allow(unused)]
use core::cell::Cell;
use critical_section::Mutex;
use embedded_hal_0_2::can::{Frame, Id, StandardId};
use mcp2515::frame::CanFrame;
use paste::paste;

pub trait GaugeLike {
    type SIZE;
    const GAUGE_ID: u8;

    fn get(&self) -> usize;
    fn set(&self, value: usize);

    fn set_from_bytes(&self, data: &[u8]) {
        let mut bytes = [0; 8];
        bytes.copy_from_slice(data);

        self.set(usize::from_be_bytes(bytes));
    }

    fn to_frame(&self) -> CanFrame {
        let data = &self.get().to_be_bytes()[..size_of::<Self::SIZE>()];
        let frame = CanFrame::new(
            Id::Standard(StandardId::new(Self::GAUGE_ID as u16).expect("bad address")),
            data,
        )
        .unwrap();

        frame
    }
}

pub struct Gauge<const ID: u8, SIZE> {
    pub value: Mutex<Cell<usize>>,
    pub size: SIZE,
}

impl<const ID: u8, SIZE> Gauge<ID, SIZE> {
    // const GAUGE_ID: u8 = ID;

    pub const fn new(initial_value: usize, size: SIZE) -> Self {
        Gauge {
            value: Mutex::new(Cell::new(initial_value)),
            size,
        }
    }
    //
    //     const fn id(&self) -> u8 {
    //         ID
    //     }
    //
    //     const fn size(&self) -> usize {
    //         size_of::<SIZE>()
    //     }
    //
    //     fn to_frame(&self) -> CanFrame {
    //         let data = &self.get().to_be_bytes()[..self.size()];
    //         let frame = CanFrame::new(
    //             Id::Standard(StandardId::new(self.id().into()).expect("bad address")),
    //             data,
    //         )
    //         .unwrap();
    //
    //         frame
    //     }
    //
    //
    //     pub fn get(&self) -> usize {
    //         let val = critical_section::with(|cs| self.value.borrow(cs).get());
    //         return val;
    //     }
    //
    //     pub fn set(&self, value: usize) {
    //         critical_section::with(|cs| self.value.borrow(cs).set(value))
    //     }
    //     pub fn deconstruct(&self) -> [u8; 2] {
    //         let val: u16 = critical_section::with(|cs| self.value.borrow(cs).get())
    //             .try_into()
    //             .unwrap();
    //         return val.to_be_bytes();
    //     }
    //     pub fn reconstruct(&self, value: [u8; 2]) {
    //         let val = u16::from_be_bytes(value);
    //         critical_section::with(|cs| self.value.borrow(cs).set(val.into()))
    //     }
}

impl<const ID: u8, SIZE> GaugeLike for Gauge<ID, SIZE> {
    type SIZE = SIZE;
    const GAUGE_ID: u8 = ID;

    fn get(&self) -> usize {
        critical_section::with(|cs| self.value.borrow(cs).get())
    }

    fn set(&self, value: usize) {
        critical_section::with(|cs| self.value.borrow(cs).set(value))
    }
}

macro_rules! gauges {
    ($($name:expr, $ty:expr),+) => {
        $(
        paste! {
            pub static $name: $ty = $ty::new(0, 0);
        }
        )+
    };
}

type StaTime = Gauge<0x20, u8>;
type StaStatus1 = Gauge<0x21, u8>; //bitfield
type StaEng = Gauge<0x22, u8>; //bitfield
type Dwell = Gauge<0x23, u8>; // x10 in ms
type Map = Gauge<0x24, u16>;
type Iat = Gauge<0x25, u8>;
type Clnt = Gauge<0x26, u8>;
type BATCorrect = Gauge<0x27, u8>; //%percent
type BATVol = Gauge<0x28, u8>;
type AFRPri = Gauge<0x29, u8>;
type EGOCorrect = Gauge<0x2A, u8>; //%percent
type IATCorrect = Gauge<0x2B, u8>; //%percent
type WUECorrect = Gauge<0x2C, u8>; //%percent
type Rpm = Gauge<0x2D, u16>;
type AccelEnrich = Gauge<0x2E, u8>; //%percent
type GammaE = Gauge<0x2F, u8>; //%percent
type Ve = Gauge<0x30, u8>; //%percent
type AFRTarget = Gauge<0x31, u8>;
type PulseWidth1 = Gauge<0x32, u16>; //Pulsewidth multiplied by 10 in ms
type TPSdot = Gauge<0x33, u8>;
type CurSparkAdvance = Gauge<0x34, u8>;
type Tps = Gauge<0x35, u8>; //%percent
type LoopPS = Gauge<0x36, u16>;
type FreeMem = Gauge<0x37, u16>;
type BoostTarget = Gauge<0x38, u8>;
type BoostPWM = Gauge<0x39, u8>;
type StaSpark = Gauge<0x3A, u8>; //bitfield
type RPMdot = Gauge<0x3B, i16>;
type EthanolPercent = Gauge<0x3C, u8>;
type FlexCorrect = Gauge<0x3D, u8>;
type FlexIgnCorrect = Gauge<0x3E, u8>;
type IdleLoad = Gauge<0x3F, u8>;
type TestOutputs = Gauge<0x40, u8>; // bitfield
type AFRSec = Gauge<0x41, u8>;
type Baro = Gauge<0x42, u8>;
type TPSadc = Gauge<0x43, u8>;
type NextError = Gauge<0x44, u8>;
type StaLaunchCorrect = Gauge<0x45, u8>;
type PulseWidth2 = Gauge<0x46, u8>;
type PulseWidth3 = Gauge<0x47, u8>;
type PulseWidth4 = Gauge<0x48, u8>;
type StaStatus2 = Gauge<0x49, u8>; // bitfield
type EngProtectSta = Gauge<0x4A, u8>; //bitfield
type FuelLoad = Gauge<0x4B, u16>;
type IgnLoad = Gauge<0x4C, u16>;
type InjAngle = Gauge<0x4D, u16>;
type IdleDuty = Gauge<0x4E, u8>;
type CLIdleTarget = Gauge<0x4F, u8>;
type MAPdot = Gauge<0x50, u8>;
type VVTAngle = Gauge<0x51, u8>;
type VVTTargetAngle = Gauge<0x52, u8>;
type VVTDuty = Gauge<0x53, u8>;
type FlexBoostCorrect = Gauge<0x54, u16>;
type BaroCorrection = Gauge<0x55, u8>;
type Ase = Gauge<0x56, u8>;
type Vss = Gauge<0x57, u16>;
type Gear = Gauge<0x58, u8>;
type FuelPres = Gauge<0x59, u8>;
type OilPres = Gauge<0x5A, u8>;
type Wmipw = Gauge<0x5B, u8>;
type StaStatus4 = Gauge<0x5C, u8>; //bitfield
type VVTAngle2 = Gauge<0x5D, u8>;
type VVTTargetAngle2 = Gauge<0x5E, u8>;
type VVTDuty2 = Gauge<0x5F, u8>;
type StatusOutSta = Gauge<0x60, u8>;
type FlexFuelTemp = Gauge<0x61, u8>;
type FuelTempCorrect = Gauge<0x62, u8>;
type Ve1 = Gauge<0x63, u8>;
type Ve2 = Gauge<0x64, u8>;
type Advance1 = Gauge<0x66, u8>;
type Advance2 = Gauge<0x67, u8>;
type NitroSta = Gauge<0x68, u8>;
type SDsta = Gauge<0x69, u8>;

enum GaugeType {}

gauges! {
    STA_TIME, StaTime,
    STA_STATUS1, StaStatus1,
    STA_ENG, StaEng,
    DWELL, Dwell,
    MAP, Map,
    IAT, Iat,
    CLNT, Clnt,
    BAT_CORRECT, BATCorrect,
    BAT_VOL, BATVol,
    AFR_PRI, AFRPri,
    EGO_CORRECT, EGOCorrect,
    IAT_CORRECT, IATCorrect,
    WUE_CORRECT, WUECorrect,
    RPM, Rpm,
    ACCEL_ENRICH, AccelEnrich,
    GAMME_E, GammaE,
    VE, Ve,
    AFR_TARGET, AFRTarget,
    PULSE_WIDTH1, PulseWidth1,
    TPS_DOT, TPSdot,
    CUR_SPARK_ADVANCE, CurSparkAdvance,
    TPS, Tps,
    LOOP_PS, LoopPS,
    FREE_MEM, FreeMem,
    BOOST_TARGET, BoostTarget,
    BOOST_PWM, BoostPWM,
    STA_SPARK, StaSpark,
    RPM_DOT, RPMdot,
    ETHANOL_PERCENT, EthanolPercent,
    FLEX_CORRECT, FlexCorrect,
    FLEX_IGN_CORRECT, FlexIgnCorrect,
    IDLE_LOAD, IdleLoad,
    TEST_OUTPUTS, TestOutputs,
    AFR_SEC, AFRSec,
    BARO, Baro,
    TPS_ADC, TPSadc,
    NEXT_ERROR, NextError,
    STA_LAUNCH_CORRECT, StaLaunchCorrect,
    PULSE_WIDTH2, PulseWidth2,
    PULSE_WIDTH3, PulseWidth3,
    PULSE_WIDTH4, PulseWidth4,
    STA_STATUS2, StaStatus2,
    ENG_PROTECT_STA, EngProtectSta,
    FUEL_LOAD, FuelLoad,
    IGN_LOAD, IgnLoad,
    INJ_ANGLE, InjAngle,
    IDLE_DUTY, IdleDuty,
    CL_IDLE_TARGET, CLIdleTarget,
    MAP_DOT, MAPdot,
    VVT_ANGLE, VVTAngle,
    VVT_TARGET_ANGLE, VVTTargetAngle,
    VVT_DUTY, VVTDuty,
    FLEX_BOOST_CORRECT, FlexBoostCorrect,
    BARO_CORRECTION, BaroCorrection,
    ASE, Ase,
    VSS, Vss,
    GEAR, Gear,
    FUEL_PRES, FuelPres,
    OIL_PRES, OilPres,
    WMI_PW, Wmipw,
    STA_STATUS4, StaStatus4,
    VVT_ANGLE2, VVTAngle2,
    VVT_TARGET_ANGLE2, VVTTargetAngle2,
    VVT_DUTY2, VVTDuty2,
    STATUS_OUT_STA, StatusOutSta,
    FLEX_FUEL_TEMP, FlexFuelTemp,
    FUEL_TEMP_CORRECT, FuelTempCorrect,
    VE1, Ve1,
    VE2, Ve2,
    ADVANCE1, Advance1,
    ADVANCE2, Advance2,
    NITRO_STA, NitroSta,
    SD_STA, SDsta

}

pub fn framemaker(id: u8, data: u8) -> CanFrame {
    let frame = CanFrame::new(
        Id::Standard(StandardId::new(id.into()).expect("bad address")),
        &[data], //gauge data out
    )
    .unwrap();
    frame
}

pub fn framemaker2(id: u8, data: u8, data2: u8) -> CanFrame {
    let frame = CanFrame::new(
        Id::Standard(StandardId::new(id.into()).expect("bad address")),
        &[data, data2], //gauge data out
    )
    .unwrap();
    frame
}

pub fn get_gauge(id: usize) -> Option<impl GaugeLike> {
    let gauge = match id {
        //supermatch sugma
        StaTime::ID => STA_TIME,
        StaStatus1::ID => STA_STATUS1,
        StaEng::ID => STA_ENG,
        Dwell::ID => DWELL,
        Map::ID => MAP,
        Iat::ID => IAT,
        Clnt::ID => CLNT,
        BATCorrect::ID => BAT_CORRECT,
        BATVol::ID => BAT_VOL,
        AFRPri::ID => AFR_PRI,
        EGOCorrect::ID => EGO_CORRECT,
        IATCorrect::ID => IAT_CORRECT,
        WUECorrect::ID => WUE_CORRECT,
        Rpm::ID => RPM,
        AccelEnrich::ID => ACCEL_ENRICH,
        GammaE::ID => GAMME_E,
        Ve::ID => VE,
        AFRTarget::ID => AFR_TARGET,
        PulseWidth1::ID => PULSE_WIDTH1,
        TPSdot::ID => TPS_DOT,
        CurSparkAdvance::ID => CUR_SPARK_ADVANCE,
        Tps::ID => TPS,
        LoopPS::ID => LOOP_PS,
        FreeMem::ID => FREE_MEM,
        BoostTarget::ID => BOOST_TARGET,
        BoostPWM::ID => BOOST_PWM,
        StaSpark::ID => STA_SPARK,
        RPMdot::ID => RPM_DOT, //was i16? deal with it later
        EthanolPercent::ID => ETHANOL_PERCENT,
        FlexCorrect::ID => FLEX_CORRECT,
        FlexIgnCorrect::ID => FLEX_IGN_CORRECT,
        IdleLoad::ID => IDLE_LOAD,
        TestOutputs::ID => TEST_OUTPUTS,
        AFRSec::ID => AFR_SEC,
        Baro::ID => BARO,
        TPSadc::ID => TPS_ADC,
        NextError::ID => NEXT_ERROR,
        StaLaunchCorrect::ID => STA_LAUNCH_CORRECT,
        PulseWidth2::ID => PULSE_WIDTH2,
        PulseWidth3::ID => PULSE_WIDTH3,
        PulseWidth4::ID => PULSE_WIDTH4,
        StaStatus2::ID => STA_STATUS2,
        EngProtectSta::ID => ENG_PROTECT_STA,
        FuelLoad::ID => FUEL_LOAD,
        IgnLoad::ID => IGN_LOAD,
        InjAngle::ID => INJ_ANGLE,
        IdleDuty::ID => IDLE_DUTY,
        CLIdleTarget::ID => CL_IDLE_TARGET,
        MAPdot::ID => MAP_DOT,
        VVTAngle::ID => VVT_ANGLE,
        VVTTargetAngle::ID => VVT_TARGET_ANGLE,
        VVTDuty::ID => VVT_DUTY,
        FlexBoostCorrect::ID => FLEX_BOOST_CORRECT,
        BaroCorrection::ID => BARO_CORRECTION,
        Ase::ID => ASE,
        Vss::ID => VSS,
        Gear::ID => GEAR,
        FuelPres::ID => FUEL_PRES,
        OilPres::ID => OIL_PRES,
        Wmipw::ID => WMI_PW,
        StaStatus4::ID => STA_STATUS4,
        VVTAngle2::ID => VVT_ANGLE2,
        VVTTargetAngle2::ID => VVT_TARGET_ANGLE2,
        VVTDuty2::ID => VVT_DUTY2,
        StatusOutSta::ID => STATUS_OUT_STA,
        FlexFuelTemp::ID => FLEX_FUEL_TEMP,
        FuelTempCorrect::ID => FUEL_TEMP_CORRECT,
        Ve1::ID => VE1,
        Ve2::ID => VE2,
        Advance1::ID => ADVANCE1,
        Advance2::ID => ADVANCE2,
        NitroSta::ID => NITRO_STA,
        SDsta::ID => SD_STA,
        _ => {
            return None;
        }
    };

    todo!("don't touch me idiot fuck you")
}

pub fn can_match(check: u8) -> Option<CanFrame> {
    //match for the master to match against its cliaskedaddr
    //vec
    /*macro_rules! gauges2 {
        ($($name:expr, $ty:expr),+) => {match i {
            $(
                &$ty::ID => {
                    let frame = framemaker($ty::ID, $name.get().try_into().unwrap());

                }

            )+
            _ => continue,
        }
        };
    }*/
    let frame = match check {
        //supermatch sugma
        StaTime::GAUGE_ID => STA_TIME.to_frame(),
        StaStatus1::GAUGE_ID => STA_STATUS1.to_frame(),
        StaEng::GAUGE_ID => STA_ENG.to_frame(),
        Dwell::GAUGE_ID => DWELL.to_frame(),
        Map::GAUGE_ID => MAP.to_frame(),
        Iat::GAUGE_ID => IAT.to_frame(),
        Clnt::GAUGE_ID => CLNT.to_frame(),
        BATCorrect::GAUGE_ID => BAT_CORRECT.to_frame(),
        BATVol::GAUGE_ID => BAT_VOL.to_frame(),
        AFRPri::GAUGE_ID => AFR_PRI.to_frame(),
        EGOCorrect::GAUGE_ID => EGO_CORRECT.to_frame(),
        IATCorrect::GAUGE_ID => IAT_CORRECT.to_frame(),
        WUECorrect::GAUGE_ID => WUE_CORRECT.to_frame(),
        Rpm::GAUGE_ID => RPM.to_frame(),
        AccelEnrich::GAUGE_ID => ACCEL_ENRICH.to_frame(),
        GammaE::GAUGE_ID => GAMME_E.to_frame(),
        Ve::GAUGE_ID => VE.to_frame(),
        AFRTarget::GAUGE_ID => AFR_TARGET.to_frame(),
        PulseWidth1::GAUGE_ID => PULSE_WIDTH1.to_frame(),
        TPSdot::GAUGE_ID => TPS_DOT.to_frame(),
        CurSparkAdvance::GAUGE_ID => CUR_SPARK_ADVANCE.to_frame(),
        Tps::GAUGE_ID => TPS.to_frame(),
        LoopPS::GAUGE_ID => LOOP_PS.to_frame(),
        FreeMem::GAUGE_ID => FREE_MEM.to_frame(),
        BoostTarget::GAUGE_ID => BOOST_TARGET.to_frame(),
        BoostPWM::GAUGE_ID => BOOST_PWM.to_frame(),
        StaSpark::GAUGE_ID => STA_SPARK.to_frame(),
        RPMdot::GAUGE_ID => RPM_DOT.to_frame(), //was i16? deal with it later
        EthanolPercent::GAUGE_ID => ETHANOL_PERCENT.to_frame(),
        FlexCorrect::GAUGE_ID => FLEX_CORRECT.to_frame(),
        FlexIgnCorrect::GAUGE_ID => FLEX_IGN_CORRECT.to_frame(),
        IdleLoad::GAUGE_ID => IDLE_LOAD.to_frame(),
        TestOutputs::GAUGE_ID => TEST_OUTPUTS.to_frame(),
        AFRSec::GAUGE_ID => AFR_SEC.to_frame(),
        Baro::GAUGE_ID => BARO.to_frame(),
        TPSadc::GAUGE_ID => TPS_ADC.to_frame(),
        NextError::GAUGE_ID => NEXT_ERROR.to_frame(),
        StaLaunchCorrect::GAUGE_ID => STA_LAUNCH_CORRECT.to_frame(),
        PulseWidth2::GAUGE_ID => PULSE_WIDTH2.to_frame(),
        PulseWidth3::GAUGE_ID => PULSE_WIDTH3.to_frame(),
        PulseWidth4::GAUGE_ID => PULSE_WIDTH4.to_frame(),
        StaStatus2::GAUGE_ID => STA_STATUS2.to_frame(),
        EngProtectSta::GAUGE_ID => ENG_PROTECT_STA.to_frame(),
        FuelLoad::GAUGE_ID => FUEL_LOAD.to_frame(),
        IgnLoad::GAUGE_ID => IGN_LOAD.to_frame(),
        InjAngle::GAUGE_ID => INJ_ANGLE.to_frame(),
        IdleDuty::GAUGE_ID => IDLE_DUTY.to_frame(),
        CLIdleTarget::GAUGE_ID => CL_IDLE_TARGET.to_frame(),
        MAPdot::GAUGE_ID => MAP_DOT.to_frame(),
        VVTAngle::GAUGE_ID => VVT_ANGLE.to_frame(),
        VVTTargetAngle::GAUGE_ID => VVT_TARGET_ANGLE.to_frame(),
        VVTDuty::GAUGE_ID => VVT_DUTY.to_frame(),
        FlexBoostCorrect::GAUGE_ID => FLEX_BOOST_CORRECT.to_frame(),
        BaroCorrection::GAUGE_ID => BARO_CORRECTION.to_frame(),
        Ase::GAUGE_ID => ASE.to_frame(),
        Vss::GAUGE_ID => VSS.to_frame(),
        Gear::GAUGE_ID => GEAR.to_frame(),
        FuelPres::GAUGE_ID => FUEL_PRES.to_frame(),
        OilPres::GAUGE_ID => OIL_PRES.to_frame(),
        Wmipw::GAUGE_ID => WMI_PW.to_frame(),
        StaStatus4::GAUGE_ID => STA_STATUS4.to_frame(),
        VVTAngle2::GAUGE_ID => VVT_ANGLE2.to_frame(),
        VVTTargetAngle2::GAUGE_ID => VVT_TARGET_ANGLE2.to_frame(),
        VVTDuty2::GAUGE_ID => VVT_DUTY2.to_frame(),
        StatusOutSta::GAUGE_ID => STATUS_OUT_STA.to_frame(),
        FlexFuelTemp::GAUGE_ID => FLEX_FUEL_TEMP.to_frame(),
        FuelTempCorrect::GAUGE_ID => FUEL_TEMP_CORRECT.to_frame(),
        Ve1::GAUGE_ID => VE1.to_frame(),
        Ve2::GAUGE_ID => VE2.to_frame(),
        Advance1::GAUGE_ID => ADVANCE1.to_frame(),
        Advance2::GAUGE_ID => ADVANCE2.to_frame(),
        NitroSta::GAUGE_ID => NITRO_STA.to_frame(),
        SDsta::GAUGE_ID => SD_STA.to_frame(),
        _ => {
            return None;
        }
    };

    return Some(frame);
}

pub fn speeduino_n_writer(buf: [u8; 126]) {
    // let mut tmprebuilder: [u8; 2];
    STA_TIME.set(buf[3] as usize);
    STA_STATUS1.set(buf[4] as usize);
    STA_ENG.set(buf[5] as usize);
    DWELL.set(buf[6] as usize);
    // tmprebuilder = [buf[7], buf[8]];
    MAP.set_from_bytes(&buf[7..=8]);
    IAT.set(buf[9] as usize);
    CLNT.set(buf[10] as usize);
    BAT_CORRECT.set(buf[11] as usize);
    BAT_VOL.set(buf[12] as usize);
    AFR_PRI.set(buf[13] as usize);
    EGO_CORRECT.set(buf[14] as usize);
    IAT_CORRECT.set(buf[15] as usize);
    WUE_CORRECT.set(buf[16] as usize);
    // tmprebuilder = [buf[17], buf[18]];
    RPM.set_from_bytes(&buf[17..=18]);
    ACCEL_ENRICH.set(buf[19] as usize);
    GAMME_E.set(buf[20] as usize);
    VE.set(buf[21] as usize);
    AFR_TARGET.set(buf[22] as usize);
    // tmprebuilder = [buf[23], buf[24]];
    PULSE_WIDTH1.set_from_bytes(&buf[23..=24]);
    TPS_DOT.set(buf[25] as usize);
    CUR_SPARK_ADVANCE.set(buf[26] as usize);
    TPS.set(buf[27] as usize);
    // tmprebuilder = [buf[28..=29]];
    LOOP_PS.set_from_bytes(&buf[28..=29]);
    // tmprebuilder = [buf[30..=31]];
    FREE_MEM.set_from_bytes(&buf[30..=31]);
    BOOST_TARGET.set(buf[32] as usize);
    BOOST_PWM.set(buf[33] as usize);
    STA_SPARK.set(buf[34] as usize);
    // tmprebuilder = [buf[35..=36]];
    RPM_DOT.set_from_bytes(&buf[35..=36]);
    ETHANOL_PERCENT.set(buf[37] as usize);
    FLEX_CORRECT.set(buf[38] as usize);
    FLEX_IGN_CORRECT.set(buf[39] as usize);
    IDLE_LOAD.set(buf[40] as usize);
    TEST_OUTPUTS.set(buf[41] as usize);
    AFR_SEC.set(buf[42] as usize);
    BARO.set(buf[43] as usize);
    TPS_ADC.set(buf[76] as usize);
    NEXT_ERROR.set(buf[77] as usize);
    STA_LAUNCH_CORRECT.set(buf[78] as usize);
    // tmprebuilder = [buf[79..=80]];
    PULSE_WIDTH2.set_from_bytes(&buf[79..=80]);
    // tmprebuilder = [buf[81..=82]];
    PULSE_WIDTH3.set_from_bytes(&buf[81..=82]);
    // tmprebuilder = [buf[83..=84]];
    PULSE_WIDTH4.set_from_bytes(&buf[83..=84]);
    STA_STATUS2.set(buf[85] as usize);
    ENG_PROTECT_STA.set(buf[86] as usize);
    // tmprebuilder = [buf[87..=88]];
    FUEL_LOAD.set_from_bytes(&buf[87..=88]);
    // tmprebuilder = [buf[89..=90]];
    IGN_LOAD.set_from_bytes(&buf[89..=90]);
    // tmprebuilder = [buf[91..=92]];
    INJ_ANGLE.set_from_bytes(&buf[91..=92]);
    IDLE_DUTY.set(buf[93] as usize);
    CL_IDLE_TARGET.set(buf[94] as usize);
    MAP_DOT.set(buf[95] as usize);
    VVT_ANGLE.set(buf[96] as usize);
    VVT_TARGET_ANGLE.set(buf[97] as usize);
    VVT_DUTY.set(buf[98] as usize);
    // tmprebuilder = [buf[99..=100]];
    FLEX_BOOST_CORRECT.set_from_bytes(&buf[99..=100]);
    BARO_CORRECTION.set(buf[101] as usize);
    ASE.set(buf[102] as usize);
    // tmprebuilder = [buf[103..=104]];
    VSS.set_from_bytes(&buf[103..=104]);
    GEAR.set(buf[105] as usize);
    FUEL_PRES.set(buf[106] as usize);
    OIL_PRES.set(buf[107] as usize);
    WMI_PW.set(buf[108] as usize);
    STA_STATUS4.set(buf[109] as usize);
    VVT_ANGLE2.set(buf[110] as usize);
    VVT_TARGET_ANGLE2.set(buf[111] as usize);
    VVT_DUTY2.set(buf[112] as usize);
    STATUS_OUT_STA.set(buf[113] as usize);
    FLEX_FUEL_TEMP.set(buf[114] as usize);
    FUEL_TEMP_CORRECT.set(buf[115] as usize);
    VE1.set(buf[116] as usize);
    VE2.set(buf[117] as usize);
    ADVANCE1.set(buf[118] as usize);
    ADVANCE2.set(buf[119] as usize);
    NITRO_STA.set(buf[120] as usize);
    SD_STA.set(buf[121] as usize);
}
#[allow(non_snake_case)]
pub fn speeduino_A_writer(buf: [u8; 126]) {
    // let mut tmprebuilder: [u8; 2];
    STA_TIME.set(buf[2] as usize);
    STA_STATUS1.set(buf[3] as usize);
    STA_ENG.set(buf[4] as usize);
    DWELL.set(buf[5] as usize);
    // tmprebuilder = [buf[6..=7]];
    MAP.set_from_bytes(&buf[6..=7]);
    IAT.set(buf[8] as usize);
    CLNT.set(buf[9] as usize);
    BAT_CORRECT.set(buf[10] as usize);
    BAT_VOL.set(buf[11] as usize);
    AFR_PRI.set(buf[12] as usize);
    EGO_CORRECT.set(buf[13] as usize);
    IAT_CORRECT.set(buf[14] as usize);
    WUE_CORRECT.set(buf[15] as usize);
    // tmprebuilder = [buf[16..=17]];
    RPM.set_from_bytes(&buf[16..=17]);
    ACCEL_ENRICH.set(buf[18] as usize);
    GAMME_E.set(buf[19] as usize);
    VE.set(buf[20] as usize);
    AFR_TARGET.set(buf[21] as usize);
    // tmprebuilder = [buf[22..=23]];
    PULSE_WIDTH1.set_from_bytes(&buf[22..=23]);
    TPS_DOT.set(buf[24] as usize);
    CUR_SPARK_ADVANCE.set(buf[25] as usize);
    TPS.set(buf[26] as usize);
    // tmprebuilder = [buf[27..=28]];
    LOOP_PS.set_from_bytes(&buf[27..=28]);
    // tmprebuilder = [buf[29..=30]];
    FREE_MEM.set_from_bytes(&buf[29..=30]);
    BOOST_TARGET.set(buf[31] as usize);
    BOOST_PWM.set(buf[32] as usize);
    STA_SPARK.set(buf[33] as usize);
    // tmprebuilder = [buf[34..=35]];
    RPM_DOT.set_from_bytes(&buf[34..=35]);
    ETHANOL_PERCENT.set(buf[36] as usize);
    FLEX_CORRECT.set(buf[37] as usize);
    FLEX_IGN_CORRECT.set(buf[38] as usize);
    IDLE_LOAD.set(buf[39] as usize);
    TEST_OUTPUTS.set(buf[40] as usize);
    AFR_SEC.set(buf[41] as usize);
    BARO.set(buf[42] as usize);
    TPS_ADC.set(buf[75] as usize);
}

pub fn cli_wri(val: usize, idnum: u8) {
    let frame = match idnum {
        //supermatch sugma
        StaTime::GAUGE_ID => STA_TIME.set(val),
        StaStatus1::GAUGE_ID => STA_STATUS1.set(val),
        StaEng::GAUGE_ID => STA_ENG.set(val),
        Dwell::GAUGE_ID => DWELL.set(val),
        Map::GAUGE_ID => MAP.set(val),
        Iat::GAUGE_ID => IAT.set(val),
        Clnt::GAUGE_ID => CLNT.set(val),
        BATCorrect::GAUGE_ID => BAT_CORRECT.set(val),
        BATVol::GAUGE_ID => BAT_VOL.set(val),
        AFRPri::GAUGE_ID => AFR_PRI.set(val),
        EGOCorrect::GAUGE_ID => EGO_CORRECT.set(val),
        IATCorrect::GAUGE_ID => IAT_CORRECT.set(val),
        WUECorrect::GAUGE_ID => WUE_CORRECT.set(val),
        Rpm::GAUGE_ID => RPM.set(val),
        AccelEnrich::GAUGE_ID => ACCEL_ENRICH.set(val),
        GammaE::GAUGE_ID => GAMME_E.set(val),
        Ve::GAUGE_ID => VE.set(val),
        AFRTarget::GAUGE_ID => AFR_TARGET.set(val),
        PulseWidth1::GAUGE_ID => PULSE_WIDTH1.set(val),
        TPSdot::GAUGE_ID => TPS_DOT.set(val),
        CurSparkAdvance::GAUGE_ID => CUR_SPARK_ADVANCE.set(val),
        Tps::GAUGE_ID => TPS.set(val),
        LoopPS::GAUGE_ID => LOOP_PS.set(val),
        FreeMem::GAUGE_ID => FREE_MEM.set(val),
        BoostTarget::GAUGE_ID => BOOST_TARGET.set(val),
        BoostPWM::GAUGE_ID => BOOST_PWM.set(val),
        StaSpark::GAUGE_ID => STA_SPARK.set(val),
        RPMdot::GAUGE_ID => RPM_DOT.set(val),
        EthanolPercent::GAUGE_ID => ETHANOL_PERCENT.set(val),
        FlexCorrect::GAUGE_ID => FLEX_CORRECT.set(val),
        FlexIgnCorrect::GAUGE_ID => FLEX_IGN_CORRECT.set(val),
        IdleLoad::GAUGE_ID => IDLE_LOAD.set(val),
        TestOutputs::GAUGE_ID => TEST_OUTPUTS.set(val),
        AFRSec::GAUGE_ID => AFR_SEC.set(val),
        Baro::GAUGE_ID => BARO.set(val),
        TPSadc::GAUGE_ID => TPS_ADC.set(val),
        NextError::GAUGE_ID => NEXT_ERROR.set(val),
        StaLaunchCorrect::GAUGE_ID => STA_LAUNCH_CORRECT.set(val),
        PulseWidth2::GAUGE_ID => PULSE_WIDTH2.set(val),
        PulseWidth3::GAUGE_ID => PULSE_WIDTH3.set(val),
        PulseWidth4::GAUGE_ID => PULSE_WIDTH4.set(val),
        StaStatus2::GAUGE_ID => STA_STATUS2.set(val),
        EngProtectSta::GAUGE_ID => ENG_PROTECT_STA.set(val),
        FuelLoad::GAUGE_ID => FUEL_LOAD.set(val),
        IgnLoad::GAUGE_ID => IGN_LOAD.set(val),
        InjAngle::GAUGE_ID => INJ_ANGLE.set(val),
        IdleDuty::GAUGE_ID => IDLE_DUTY.set(val),
        CLIdleTarget::GAUGE_ID => CL_IDLE_TARGET.set(val),
        MAPdot::GAUGE_ID => MAP_DOT.set(val),
        VVTAngle::GAUGE_ID => VVT_ANGLE.set(val),
        VVTTargetAngle::GAUGE_ID => VVT_TARGET_ANGLE.set(val),
        VVTDuty::GAUGE_ID => VVT_DUTY.set(val),
        FlexBoostCorrect::GAUGE_ID => FLEX_BOOST_CORRECT.set(val),
        BaroCorrection::GAUGE_ID => BARO_CORRECTION.set(val),
        Ase::GAUGE_ID => ASE.set(val),
        Vss::GAUGE_ID => VSS.set(val),
        Gear::GAUGE_ID => GEAR.set(val),
        FuelPres::GAUGE_ID => FUEL_PRES.set(val),
        OilPres::GAUGE_ID => OIL_PRES.set(val),
        Wmipw::GAUGE_ID => WMI_PW.set(val),
        StaStatus4::GAUGE_ID => STA_STATUS4.set(val),
        VVTAngle2::GAUGE_ID => VVT_ANGLE2.set(val),
        VVTTargetAngle2::GAUGE_ID => VVT_TARGET_ANGLE2.set(val),
        VVTDuty2::GAUGE_ID => VVT_DUTY2.set(val),
        StatusOutSta::GAUGE_ID => STATUS_OUT_STA.set(val),
        FlexFuelTemp::GAUGE_ID => FLEX_FUEL_TEMP.set(val),
        FuelTempCorrect::GAUGE_ID => FUEL_TEMP_CORRECT.set(val),
        Ve1::GAUGE_ID => VE1.set(val),
        Ve2::GAUGE_ID => VE2.set(val),
        Advance1::GAUGE_ID => ADVANCE1.set(val),
        Advance2::GAUGE_ID => ADVANCE2.set(val),
        NitroSta::GAUGE_ID => NITRO_STA.set(val),
        SDsta::GAUGE_ID => SD_STA.set(val),
        _ => (),
    };
}
