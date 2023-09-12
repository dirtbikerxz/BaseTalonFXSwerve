package frc.lib.fault;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

public  class FaultGrabber {
    public static FaultList grabTalonFX(TalonFX mc) {
        FaultList faults = new FaultList();
        faults.add(new Fault("BootDuringEnable", () -> mc.getFault_BootDuringEnable().getValue(), () -> mc.getStickyFault_BootDuringEnable().getValue()));
        faults.add(new Fault("DeviceTemp", () -> mc.getFault_DeviceTemp().getValue(), () -> mc.getStickyFault_DeviceTemp().getValue()));
        faults.add(new Fault("ForwardHardLimit", () -> mc.getFault_ForwardHardLimit().getValue(), () -> mc.getStickyFault_ForwardHardLimit().getValue()));
        faults.add(new Fault("ForwardSoftLimit", () -> mc.getFault_ForwardSoftLimit().getValue(), () -> mc.getStickyFault_ForwardSoftLimit().getValue()));
        faults.add(new Fault("FusedSensorOutOfSync", () -> mc.getFault_FusedSensorOutOfSync().getValue(), () -> mc.getStickyFault_FusedSensorOutOfSync().getValue()));
        faults.add(new Fault("Hardware", () -> mc.getFault_Hardware().getValue(), () -> mc.getStickyFault_Hardware().getValue()));
        faults.add(new Fault("MissingRemoteSensor", () -> mc.getFault_MissingRemoteSensor().getValue(), () -> mc.getStickyFault_MissingRemoteSensor().getValue()));
        faults.add(new Fault("OverSupplyV", () -> mc.getFault_OverSupplyV().getValue(), () -> mc.getStickyFault_OverSupplyV().getValue()));
        faults.add(new Fault("ProcTemp", () -> mc.getFault_ProcTemp().getValue(), () -> mc.getStickyFault_ProcTemp().getValue()));
        faults.add(new Fault("ReverseHardLimit", () -> mc.getFault_ReverseHardLimit().getValue(), () -> mc.getStickyFault_ReverseHardLimit().getValue()));
        faults.add(new Fault("ReverseSoftLimit", () -> mc.getFault_ReverseSoftLimit().getValue(), () -> mc.getStickyFault_ReverseSoftLimit().getValue()));
        faults.add(new Fault("StatorCurrLimit", () -> mc.getFault_StatorCurrLimit().getValue(), () -> mc.getStickyFault_StatorCurrLimit().getValue()));
        faults.add(new Fault("SupplyCurrLimit", () -> mc.getFault_SupplyCurrLimit().getValue(), () -> mc.getStickyFault_SupplyCurrLimit().getValue()));
        faults.add(new Fault("Undervoltage", () -> mc.getFault_Undervoltage().getValue(), () -> mc.getStickyFault_Undervoltage().getValue()));
        faults.add(new Fault("UnlicensedFeatureInUse", () -> mc.getFault_UnlicensedFeatureInUse().getValue(), () -> mc.getStickyFault_UnlicensedFeatureInUse().getValue()));
        faults.add(new Fault("UnstableSupplyV", () -> mc.getFault_UnstableSupplyV().getValue(), () -> mc.getStickyFault_UnstableSupplyV().getValue()));
        faults.add(new Fault("UsingFusedCancoderWhileUnlicensed", () -> mc.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue(), () -> mc.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue()));
        
        return faults;
    }

    public static FaultList grabCANcoder(CANcoder cancoder) {
        FaultList faults = new FaultList();

        faults.add(new Fault("BadMagnet", () -> cancoder.getFault_BadMagnet().getValue(), () -> cancoder.getStickyFault_BadMagnet().getValue()));
        faults.add(new Fault("BootDuringEnable", () -> cancoder.getFault_BootDuringEnable().getValue(), () -> cancoder.getStickyFault_BootDuringEnable().getValue()));
        faults.add(new Fault("Hardware", () -> cancoder.getFault_Hardware().getValue(), () -> cancoder.getStickyFault_Hardware().getValue()));
        faults.add(new Fault("Undervoltage", () -> cancoder.getFault_Undervoltage().getValue(), () -> cancoder.getStickyFault_Undervoltage().getValue()));
        faults.add(new Fault("UnlicensedFeatureInUse", () -> cancoder.getFault_UnlicensedFeatureInUse().getValue(), () -> cancoder.getStickyFault_UnlicensedFeatureInUse().getValue()));
        return faults;
    }

    public static FaultList grabPigeon(Pigeon2 pigeon) {
        FaultList faults = new FaultList();

        faults.add(new Fault("BootDuringEnable", () -> pigeon.getFault_BootDuringEnable().getValue(), () -> pigeon.getStickyFault_BootDuringEnable().getValue()));
        faults.add(new Fault("BootIntoMotion", () -> pigeon.getFault_BootIntoMotion().getValue(), () -> pigeon.getStickyFault_BootIntoMotion().getValue()));
        faults.add(new Fault("BootupAccelerometer", () -> pigeon.getFault_BootupAccelerometer().getValue(), () -> pigeon.getStickyFault_BootupAccelerometer().getValue()));
        faults.add(new Fault("BootupGyroscope", () -> pigeon.getFault_BootupGyroscope().getValue(), () -> pigeon.getStickyFault_BootupGyroscope().getValue()));
        faults.add(new Fault("BootupMagnetometer", () -> pigeon.getFault_BootupMagnetometer().getValue(), () -> pigeon.getStickyFault_BootupMagnetometer().getValue()));
        faults.add(new Fault("DataAquiredLate", () -> pigeon.getFault_DataAcquiredLate().getValue(), () -> pigeon.getStickyFault_DataAcquiredLate().getValue()));
        faults.add(new Fault("Hardware", () -> pigeon.getFault_Hardware().getValue(), () -> pigeon.getStickyFault_Hardware().getValue()));
        faults.add(new Fault("LoopTimeSlow", () -> pigeon.getFault_LoopTimeSlow().getValue(), () -> pigeon.getStickyFault_LoopTimeSlow().getValue()));
        faults.add(new Fault("SaturatedAccelometer", () -> pigeon.getFault_SaturatedAccelometer().getValue(), () -> pigeon.getStickyFault_SaturatedAccelometer().getValue()));
        faults.add(new Fault("SaturatedGyrosscope", () -> pigeon.getFault_SaturatedGyrosscope().getValue(), () -> pigeon.getStickyFault_SaturatedGyrosscope().getValue()));
        faults.add(new Fault("SaturatedMagneter", () -> pigeon.getFault_SaturatedMagneter().getValue(), () -> pigeon.getStickyFault_SaturatedMagneter().getValue()));
        faults.add(new Fault("Undervoltage", () -> pigeon.getFault_Undervoltage().getValue(), () -> pigeon.getStickyFault_Undervoltage().getValue()));
        faults.add(new Fault("UnlicensedFeatureInUse", () -> pigeon.getFault_UnlicensedFeatureInUse().getValue(), () -> pigeon.getStickyFault_UnlicensedFeatureInUse().getValue()));

        return faults;
    }

    
    public static FaultList grabRio() {
        FaultList faults = new FaultList();

        faults.add(new Fault("UserButton", () -> RobotController.getUserButton()));
        faults.add(new Fault("BrownedOut", () -> RobotController.isBrownedOut()));
        faults.add(new Fault("CANBusOff", () -> RobotController.getCANStatus().busOffCount));
        faults.add(new Fault("CANReceiveError", () -> RobotController.getCANStatus().receiveErrorCount));
        faults.add(new Fault("CANTransmitError", () -> RobotController.getCANStatus().transmitErrorCount));
        faults.add(new Fault("Fault3V3", () -> RobotController.getFaultCount3V3()));
        faults.add(new Fault("Fault5V", () -> RobotController.getFaultCount5V()));
        faults.add(new Fault("Fault6V", () -> RobotController.getFaultCount6V()));

        return faults;
    }

    public static FaultList grabPDH(PowerDistribution pdh) {
        FaultList faults = new FaultList();

        faults.add(new Fault("Brownout", () -> pdh.getFaults().Brownout, () -> pdh.getStickyFaults().Brownout));
        faults.add(new Fault("CanWarning", () -> pdh.getFaults().CanWarning, () -> pdh.getStickyFaults().CanWarning));
        faults.add(new Fault("Channel0BreakerFault", () -> pdh.getFaults().Channel0BreakerFault, () -> pdh.getStickyFaults().Channel0BreakerFault));
        faults.add(new Fault("Channel1BreakerFault", () -> pdh.getFaults().Channel1BreakerFault, () -> pdh.getStickyFaults().Channel1BreakerFault));
        faults.add(new Fault("Channel2BreakerFault", () -> pdh.getFaults().Channel2BreakerFault, () -> pdh.getStickyFaults().Channel2BreakerFault));
        faults.add(new Fault("Channel3BreakerFault", () -> pdh.getFaults().Channel3BreakerFault, () -> pdh.getStickyFaults().Channel3BreakerFault));
        faults.add(new Fault("Channel4BreakerFault", () -> pdh.getFaults().Channel4BreakerFault, () -> pdh.getStickyFaults().Channel4BreakerFault));
        faults.add(new Fault("Channel5BreakerFault", () -> pdh.getFaults().Channel5BreakerFault, () -> pdh.getStickyFaults().Channel5BreakerFault));
        faults.add(new Fault("Channel6BreakerFault", () -> pdh.getFaults().Channel6BreakerFault, () -> pdh.getStickyFaults().Channel6BreakerFault));
        faults.add(new Fault("Channel7BreakerFault", () -> pdh.getFaults().Channel7BreakerFault, () -> pdh.getStickyFaults().Channel7BreakerFault));
        faults.add(new Fault("Channel8BreakerFault", () -> pdh.getFaults().Channel8BreakerFault, () -> pdh.getStickyFaults().Channel8BreakerFault));
        faults.add(new Fault("Channel9BreakerFault", () -> pdh.getFaults().Channel9BreakerFault, () -> pdh.getStickyFaults().Channel9BreakerFault));
        faults.add(new Fault("Channel10BreakerFault", () -> pdh.getFaults().Channel10BreakerFault, () -> pdh.getStickyFaults().Channel10BreakerFault));
        faults.add(new Fault("Channel11BreakerFault", () -> pdh.getFaults().Channel11BreakerFault, () -> pdh.getStickyFaults().Channel11BreakerFault));
        faults.add(new Fault("Channel12BreakerFault", () -> pdh.getFaults().Channel12BreakerFault, () -> pdh.getStickyFaults().Channel12BreakerFault));
        faults.add(new Fault("Channel13BreakerFault", () -> pdh.getFaults().Channel13BreakerFault, () -> pdh.getStickyFaults().Channel13BreakerFault));
        faults.add(new Fault("Channel14BreakerFault", () -> pdh.getFaults().Channel14BreakerFault, () -> pdh.getStickyFaults().Channel14BreakerFault));
        faults.add(new Fault("Channel15BreakerFault", () -> pdh.getFaults().Channel15BreakerFault, () -> pdh.getStickyFaults().Channel15BreakerFault));
        faults.add(new Fault("Channel16BreakerFault", () -> pdh.getFaults().Channel16BreakerFault, () -> pdh.getStickyFaults().Channel16BreakerFault));
        faults.add(new Fault("Channel17BreakerFault", () -> pdh.getFaults().Channel17BreakerFault, () -> pdh.getStickyFaults().Channel17BreakerFault));
        faults.add(new Fault("Channel18BreakerFault", () -> pdh.getFaults().Channel18BreakerFault, () -> pdh.getStickyFaults().Channel18BreakerFault));
        faults.add(new Fault("Channel19BreakerFault", () -> pdh.getFaults().Channel19BreakerFault, () -> pdh.getStickyFaults().Channel19BreakerFault));
        faults.add(new Fault("Channel20BreakerFault", () -> pdh.getFaults().Channel20BreakerFault, () -> pdh.getStickyFaults().Channel20BreakerFault));
        faults.add(new Fault("Channel21BreakerFault", () -> pdh.getFaults().Channel21BreakerFault, () -> pdh.getStickyFaults().Channel21BreakerFault));
        faults.add(new Fault("Channel22BreakerFault", () -> pdh.getFaults().Channel22BreakerFault, () -> pdh.getStickyFaults().Channel22BreakerFault));
        faults.add(new Fault("Channel23BreakerFault", () -> pdh.getFaults().Channel23BreakerFault, () -> pdh.getStickyFaults().Channel23BreakerFault));
        faults.add(new Fault("HardwareFault", () -> pdh.getFaults().HardwareFault));
        faults.add(new Fault("CanBusOff", () -> false, () -> pdh.getStickyFaults().CanBusOff));
        faults.add(new Fault("HasReset", () -> false, () -> pdh.getStickyFaults().HasReset));

        return faults;
    }
    
    public FaultList grabPDP(PowerDistribution pdp) { return grabPDH(pdp); }

    public FaultList grabPH(PneumaticHub ph) {
        FaultList faults = new FaultList();

        faults.add(new Fault("Brownout", () -> ph.getFaults().Brownout, () -> ph.getStickyFaults().Brownout));
        faults.add(new Fault("CanWarning", () -> ph.getFaults().CanWarning, () -> ph.getStickyFaults().CanWarning));
        faults.add(new Fault("Channel0Fault", () -> ph.getFaults().Channel0Fault));
        faults.add(new Fault("Channel1Fault", () -> ph.getFaults().Channel1Fault));
        faults.add(new Fault("Channel2Fault", () -> ph.getFaults().Channel2Fault));
        faults.add(new Fault("Channel3Fault", () -> ph.getFaults().Channel3Fault));
        faults.add(new Fault("Channel4Fault", () -> ph.getFaults().Channel4Fault));
        faults.add(new Fault("Channel5Fault", () -> ph.getFaults().Channel5Fault));
        faults.add(new Fault("Channel6Fault", () -> ph.getFaults().Channel6Fault));
        faults.add(new Fault("Channel7Fault", () -> ph.getFaults().Channel7Fault));
        faults.add(new Fault("Channel8Fault", () -> ph.getFaults().Channel8Fault));
        faults.add(new Fault("Channel9Fault", () -> ph.getFaults().Channel9Fault));
        faults.add(new Fault("Channel10Fault", () -> ph.getFaults().Channel10Fault));
        faults.add(new Fault("Channel11Fault", () -> ph.getFaults().Channel11Fault));
        faults.add(new Fault("Channel12Fault", () -> ph.getFaults().Channel12Fault));
        faults.add(new Fault("Channel13Fault", () -> ph.getFaults().Channel13Fault));
        faults.add(new Fault("Channel14Fault", () -> ph.getFaults().Channel14Fault));
        faults.add(new Fault("Channel15Fault", () -> ph.getFaults().Channel15Fault));
        faults.add(new Fault("CompressorOpen", () -> ph.getFaults().CompressorOpen, () -> ph.getStickyFaults().CompressorOpen));
        faults.add(new Fault("CompressorOverCurrent", () -> ph.getFaults().CompressorOverCurrent, () -> ph.getStickyFaults().CompressorOverCurrent));
        faults.add(new Fault("HardwareFault", () -> ph.getFaults().HardwareFault));
        faults.add(new Fault("SolenoidOverCurrent", () -> ph.getFaults().SolenoidOverCurrent, () -> ph.getStickyFaults().SolenoidOverCurrent));
        faults.add(new Fault("CanBusOff", () -> false, () -> ph.getStickyFaults().CanBusOff));
        faults.add(new Fault("HasReset", () -> false, () -> ph.getStickyFaults().HasReset));


        return faults;
    }

    public FaultList grabPCM(PneumaticsControlModule pcm) {
        FaultList faults = new FaultList();

        faults.add(new Fault("CompressorCurrentTooHigh", () -> pcm.getCompressorCurrentTooHighFault(), () -> pcm.getCompressorCurrentTooHighStickyFault()));
        faults.add(new Fault("CompressorNotConnected", () -> pcm.getCompressorNotConnectedFault(), () -> pcm.getCompressorNotConnectedStickyFault()));
        faults.add(new Fault("CompressorShorted", () -> pcm.getCompressorShortedFault(), () -> pcm.getCompressorShortedStickyFault()));
        faults.add(new Fault("SolenoidVoltage", () -> pcm.getSolenoidVoltageFault(), () -> pcm.getSolenoidVoltageStickyFault()));

        return faults;
    }

    public FaultList grabSparkMax(CANSparkMax sparkmax) {
        FaultList faults = new FaultList();


        faults.add(new Fault("Brownout", () -> sparkmax.getFault(FaultID.kBrownout), () -> sparkmax.getStickyFault(FaultID.kBrownout)));
        faults.add(new Fault("CANRX", () -> sparkmax.getFault(FaultID.kCANRX), () -> sparkmax.getStickyFault(FaultID.kCANRX)));
        faults.add(new Fault("CANTX", () -> sparkmax.getFault(FaultID.kCANTX), () -> sparkmax.getStickyFault(FaultID.kCANTX)));
        faults.add(new Fault("DRVFault", () -> sparkmax.getFault(FaultID.kDRVFault), () -> sparkmax.getStickyFault(FaultID.kDRVFault)));
        faults.add(new Fault("EEPROMCRC", () -> sparkmax.getFault(FaultID.kEEPROMCRC), () -> sparkmax.getStickyFault(FaultID.kEEPROMCRC)));
        faults.add(new Fault("HardLimitFwd", () -> sparkmax.getFault(FaultID.kHardLimitFwd), () -> sparkmax.getStickyFault(FaultID.kHardLimitFwd)));
        faults.add(new Fault("HardLimitRev", () -> sparkmax.getFault(FaultID.kHardLimitRev), () -> sparkmax.getStickyFault(FaultID.kHardLimitRev)));
        faults.add(new Fault("HasReset", () -> sparkmax.getFault(FaultID.kHasReset), () -> sparkmax.getStickyFault(FaultID.kHasReset)));
        faults.add(new Fault("IWDTReset", () -> sparkmax.getFault(FaultID.kIWDTReset), () -> sparkmax.getStickyFault(FaultID.kIWDTReset)));
        faults.add(new Fault("MotorFault", () -> sparkmax.getFault(FaultID.kMotorFault), () -> sparkmax.getStickyFault(FaultID.kMotorFault)));
        faults.add(new Fault("OtherFault", () -> sparkmax.getFault(FaultID.kOtherFault), () -> sparkmax.getStickyFault(FaultID.kOtherFault)));
        faults.add(new Fault("OverCurrent", () -> sparkmax.getFault(FaultID.kOvercurrent), () -> sparkmax.getStickyFault(FaultID.kOvercurrent)));
        faults.add(new Fault("SensorFault", () -> sparkmax.getFault(FaultID.kSensorFault), () -> sparkmax.getStickyFault(FaultID.kSensorFault)));
        faults.add(new Fault("SoftLimitFwd", () -> sparkmax.getFault(FaultID.kSoftLimitFwd), () -> sparkmax.getStickyFault(FaultID.kSoftLimitFwd)));
        faults.add(new Fault("SoftLimitRev", () -> sparkmax.getFault(FaultID.kSoftLimitRev), () -> sparkmax.getStickyFault(FaultID.kSoftLimitRev)));
        faults.add(new Fault("Stall", () -> sparkmax.getFault(FaultID.kStall), () -> sparkmax.getStickyFault(FaultID.kStall)));

        return faults;
    }

    public FaultList grabColorSensor(ColorSensorV3 colorsensor) {
        FaultList faults = new FaultList();

        faults.add(new Fault("hasReset", () -> false, () -> colorsensor.hasReset()));
        faults.add(new Fault("isConnected", () -> colorsensor.isConnected()));

        return faults;
    }
}
