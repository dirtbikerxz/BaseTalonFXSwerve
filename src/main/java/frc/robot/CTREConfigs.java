package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        MotorOutputConfigs angleMotorOutput = swerveAngleFXConfig.MotorOutput;
        angleMotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        angleMotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;
        
        /* Current Limiting */
        CurrentLimitsConfigs angleCurrentLimits = swerveAngleFXConfig.CurrentLimits;
        angleCurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        angleCurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        angleCurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        angleCurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        Slot0Configs angleSlot0 = swerveAngleFXConfig.Slot0;
        angleSlot0.kP = Constants.Swerve.angleKP;
        angleSlot0.kI = Constants.Swerve.angleKI;
        angleSlot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        var driveMotorOutput = swerveDriveFXConfig.MotorOutput;
        driveMotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        driveMotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Current Limiting */
        var driveCurrentLimits = swerveDriveFXConfig.CurrentLimits;
        driveCurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveCurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        driveCurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        driveCurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        var driveSlot0 = swerveDriveFXConfig.Slot0;
        driveSlot0.kP = Constants.Swerve.driveKP;
        driveSlot0.kI = Constants.Swerve.driveKI;
        driveSlot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
    }
}