package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.subsystems.swerve.falcon.CTRESwerveConfig;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            CTRESwerveConfig.angleEnableCurrentLimit, 
            CTRESwerveConfig.angleContinuousCurrentLimit, 
            CTRESwerveConfig.anglePeakCurrentLimit, 
            CTRESwerveConfig.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = CTRESwerveConfig.angleKP;
        swerveAngleFXConfig.slot0.kI = CTRESwerveConfig.angleKI;
        swerveAngleFXConfig.slot0.kD = CTRESwerveConfig.angleKD;
        swerveAngleFXConfig.slot0.kF = CTRESwerveConfig.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            CTRESwerveConfig.driveEnableCurrentLimit, 
            CTRESwerveConfig.driveContinuousCurrentLimit, 
            CTRESwerveConfig.drivePeakCurrentLimit, 
            CTRESwerveConfig.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = CTRESwerveConfig.driveKP;
        swerveDriveFXConfig.slot0.kI = CTRESwerveConfig.driveKI;
        swerveDriveFXConfig.slot0.kD = CTRESwerveConfig.driveKD;
        swerveDriveFXConfig.slot0.kF = CTRESwerveConfig.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = CTRESwerveConfig.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = CTRESwerveConfig.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = CTRESwerveConfig.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}