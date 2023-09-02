package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();


        //Azimuth Config
        swerveAngleFXConfig.MotorOutput.Inverted = DrivetrainConstants.ANGLE_INVERT;
        swerveAngleFXConfig.MotorOutput.NeutralMode = DrivetrainConstants.ANGLE_NEUTRAL;

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.DRIVE_SUPPLY_LIMIT;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = DrivetrainConstants.DRIVE_SUPPLY_THRESHOLD;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = DrivetrainConstants.DRIVE_SUPPLY_TIMEOUT;

        swerveAngleFXConfig.Slot0.kP = DrivetrainConstants.AzimuthGains.kP;
        swerveAngleFXConfig.Slot0.kI = DrivetrainConstants.AzimuthGains.kI;
        swerveAngleFXConfig.Slot0.kD = DrivetrainConstants.AzimuthGains.kD;
        swerveAngleFXConfig.Slot0.kS = DrivetrainConstants.AzimuthGains.kS;

        swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 1;
        swerveAngleFXConfig.Feedback.RotorToSensorRatio = DrivetrainConstants.ANGLE_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;


        //Drive Config
        swerveDriveFXConfig.MotorOutput.Inverted = DrivetrainConstants.DRIVE_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = DrivetrainConstants.DRIVE_NEUTRAL;

        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = DrivetrainConstants.AZIMUTH_SUPPLY_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = DrivetrainConstants.AZIMUTH_SUPPLY_THRESHOLD;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = DrivetrainConstants.DRIVE_SUPPLY_TIMEOUT;


        swerveDriveFXConfig.Slot0.kP = DrivetrainConstants.DriveGains.kP;
        swerveDriveFXConfig.Slot0.kI = DrivetrainConstants.DriveGains.kI;
        swerveDriveFXConfig.Slot0.kD = DrivetrainConstants.DriveGains.kD;
        swerveDriveFXConfig.Slot0.kS = DrivetrainConstants.DriveGains.kS;   

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = DrivetrainConstants.DRIVE_OPEN_RAMP_RATE;
        swerveDriveFXConfig.OpenLoopRamps.TorqueOpenLoopRampPeriod = DrivetrainConstants.DRIVE_OPEN_RAMP_RATE;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = DrivetrainConstants.DRIVE_OPEN_RAMP_RATE;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DrivetrainConstants.DRIVE_CLOSED_RAMP_RATE;
        swerveDriveFXConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DrivetrainConstants.DRIVE_CLOSED_RAMP_RATE;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DrivetrainConstants.DRIVE_CLOSED_RAMP_RATE;
        

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    }
}