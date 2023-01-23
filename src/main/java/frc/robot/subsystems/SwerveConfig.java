package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Contains all the hardcoded constants needed to configure our specific
 * swerve drive, plus factory methods for creating motors and encoders.
 */
public class SwerveConfig {

    // max speeds
    public static final double maxSpeed = Units.feetToMeters(2);
    public static final double maxAngularVelocity = Units.degreesToRadians(120);

    // physical properties of chassis
    public static final double trackWidth = Units.inchesToMeters(18.75);
    public static final double wheelBase = Units.inchesToMeters(29.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = Math.PI * wheelDiameter;

    // drive train information
    public static final double driveGearRatio = (6.75 / 1.0);
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

    // drive train characterization (divide SYSID by 12 to convert from V -> % output for CTRE)
    public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    // swerve drive modules
    public static final ModuleConfig frontLeft = new ModuleConfig(10, 11, 12, -239.590);
    public static final ModuleConfig frontRight = new ModuleConfig(13, 14, 15, 206.982);
    public static final ModuleConfig backLeft = new ModuleConfig(16, 17, 18, -50.01);
    public static final ModuleConfig backRight = new ModuleConfig(19, 20, 21, 157.148);

    // swerve drive kinematics (same order as above)
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /**
     * Create a new drive motor. This is where all of the drive motor 
     * configuration is hardcoded.
     */
    public static TalonFX createDriveMotor(int canId) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = 0.05;
        config.slot0.kI = 0.0;
        config.slot0.kD = 0.0;
        config.slot0.kF = 0.0;
        config.openloopRamp = 0.25;
        config.closedloopRamp = 0.0;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 
                35, 
                60, 
                0.1);

        TalonFX motor = new TalonFX(canId);
        motor.configFactoryDefault();
        motor.configAllSettings(config);
        motor.setInverted(false);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSelectedSensorPosition(0);
        return motor;
    }

    /**
     * Create a new angle motor. This is where all of the angle motor 
     * configuration is hardcoded.
     */
    public static TalonFX createAngleMotor(int canId) {

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = 0.3;
        config.slot0.kI = 0.0;
        config.slot0.kD = 0.0;
        config.slot0.kF = 0.0;
        config.openloopRamp = 0.25;
        config.closedloopRamp = 0.0;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 
                25, 
                40, 
                0.1);

        TalonFX motor = new TalonFX(canId);
        motor.configFactoryDefault();
        motor.configAllSettings(config);
        motor.setInverted(true);
        motor.setNeutralMode(NeutralMode.Coast);
        return motor;
    }

    /**
     * Create a new angle encoder. This is where all of the angle encoder 
     * configuration is hardcoded.
     */
    public static CANCoder createAngleEncoder(int canId) {

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = false;
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

        public ModuleConfig(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = Rotation2d.fromDegrees(angleOffset);
        }
    }
}
