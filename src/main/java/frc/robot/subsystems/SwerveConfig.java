package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class SwerveConfig {
        

    /**
     * Create a new drive motor
     */
    public static TalonFX createDriveMotor(int canId) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = Constants.Swerve.driveKP;
        config.slot0.kI = Constants.Swerve.driveKI;
        config.slot0.kD = Constants.Swerve.driveKD;
        config.slot0.kF = Constants.Swerve.driveKF;        
        config.openloopRamp = Constants.Swerve.openLoopRamp;
        config.closedloopRamp = Constants.Swerve.closedLoopRamp;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.driveEnableCurrentLimit, 
                Constants.Swerve.driveContinuousCurrentLimit, 
                Constants.Swerve.drivePeakCurrentLimit, 
                Constants.Swerve.drivePeakCurrentDuration);;

        TalonFX motor = new TalonFX(canId);
        motor.configFactoryDefault();
        motor.configAllSettings(config);
        motor.setInverted(Constants.Swerve.driveMotorInvert);
        motor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    /**
     * Create a new angle motor
     */
    public static TalonFX createAngleMotor(int canId) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = Constants.Swerve.angleKP;
        config.slot0.kI = Constants.Swerve.angleKI;
        config.slot0.kD = Constants.Swerve.angleKD;
        config.slot0.kF = Constants.Swerve.angleKF;
        config.openloopRamp = Constants.Swerve.openLoopRamp;
        config.closedloopRamp = Constants.Swerve.closedLoopRamp;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                Constants.Swerve.angleEnableCurrentLimit, 
                Constants.Swerve.angleContinuousCurrentLimit, 
                Constants.Swerve.anglePeakCurrentLimit, 
                Constants.Swerve.anglePeakCurrentDuration);;

        TalonFX motor = new TalonFX(canId);
        motor.configFactoryDefault();
        motor.configAllSettings(config);
        motor.setInverted(Constants.Swerve.angleMotorInvert);
        motor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        return motor;
    }

    /**
     * Create a new angle encoder
     */

    public static CANCoder createAngleEncoder(int canId) {

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = Constants.Swerve.canCoderInvert;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;

        CANCoder encoder = new CANCoder(canId);
        encoder.configFactoryDefault();
        encoder.configAllSettings(config);
        return encoder;
    }

    /**
     * Holds settings for a single swerve module
     */
    public static class ModuleConfig {
        
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final Rotation2d angleOffset;

        public ModuleConfig(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }
}
