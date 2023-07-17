package frc.robot.subsystems.swerve.rev;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;


public class RevSwerveConfig 
{
    
    public CANCoderConfiguration canCoderConfig;

    //
    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    public static final double drivePower = 1;
    public static final double anglePower = .9;


    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =  
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23.75); 
    public static final double wheelBase = Units.inchesToMeters(23.75); 
    public static final double wheelCircumference = chosenModule.wheelCircumference;


    /* Swerve Kinematics 
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
     public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));


    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // encoder setup
    // meters per rotation
    public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio );
    public static final double driveRpmToMetersPerSecond = driveRevToMeters/60 ;
    // the number of degrees that a single rotation of the turn motor turns the wheel.
    public static final double DegreesPerTurnRotation = 360/angleGearRatio;

    
    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.05;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.2; 
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values 
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.32 / 12); 
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.2;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; 
   

 
 
   

    public RevSwerveConfig()
    {
        canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.sensorDirection = canCoderInvert;
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}

