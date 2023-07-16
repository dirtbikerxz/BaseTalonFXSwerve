package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.constants.CTRESwerveConstants;

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
            CTRESwerveConstants.Swerve.angleEnableCurrentLimit, 
            CTRESwerveConstants.Swerve.angleContinuousCurrentLimit, 
            CTRESwerveConstants.Swerve.anglePeakCurrentLimit, 
            CTRESwerveConstants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = CTRESwerveConstants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = CTRESwerveConstants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = CTRESwerveConstants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = CTRESwerveConstants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            CTRESwerveConstants.Swerve.driveEnableCurrentLimit, 
            CTRESwerveConstants.Swerve.driveContinuousCurrentLimit, 
            CTRESwerveConstants.Swerve.drivePeakCurrentLimit, 
            CTRESwerveConstants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = CTRESwerveConstants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = CTRESwerveConstants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = CTRESwerveConstants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = CTRESwerveConstants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = CTRESwerveConstants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = CTRESwerveConstants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = CTRESwerveConstants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}