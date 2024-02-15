package frc.lib;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.ModuleConstants;
import frc.lib.SwerveModuleConstants;

public class doubleNeoConstants {
    public static double stickDeadband = 0.1;


    public static final class Swerve {
        public final static int pigeonID = 30;

        public  final static ModuleConstants drivemodule =  //TODO: This must be tuned to specific robot
                ModuleConstants.SDS.MK4i.DoubleFalcon500(ModuleConstants.SDS.MK4i.driveRatio.L1and16t);

        public  final static ModuleConstants anglemodule =  //TODO: This must be tuned to specific robot
                ModuleConstants.SDS.MK4i.DoubleFalcon500(ModuleConstants.SDS.MK4i.driveRatio.L1and16t);

        /* Drivetrain Constants */
        public final static double trackWidth = Units.inchesToMeters(20.75); //TODO: This must be tuned to specific robot
        public final static double wheelBase = Units.inchesToMeters(20.75); //TODO: This must be tuned to specific robot
        public final static double wheelCircumference = drivemodule.wheelCircumference;

        /* Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public final static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public final static double driveGearRatio = drivemodule.driveGearRatio;
        public final static double angleGearRatio = anglemodule.angleGearRatio;

        /* Motor Inverts */
        public final static boolean angleMotorInvert = anglemodule.angleMotorInvert;
        public final static boolean driveMotorInvert = drivemodule.driveMotorInvert;

        /* Angle Encoder Invert */
        public final static SensorDirectionValue cancoderInvert = anglemodule.cancoderInvert;

        /* Swerve Current Limiting */
        public final static int angleCurrentLimit = 25;
        public final static int angleCurrentThreshold = 40;
        public final static double angleCurrentThresholdTime = 0.1;
        public final static boolean angleEnableCurrentLimit = true;

        public final static int driveCurrentLimit = 35;
        public final static int driveCurrentThreshold = 60;
        public final static double driveCurrentThresholdTime = 0.1;
        public final static boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public final static double openLoopRamp = 0.25;
        public final static double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public final static double angleKP = anglemodule.angleKP;
        public final static double angleKI = anglemodule.angleKI;
        public final static double angleKD = anglemodule.angleKD;

        /* Drive Motor PID Values */
        public final static double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public final static double driveKI = 0.0;
        public final static double driveKD = 0.0;
        public final static double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public final static double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public final static double driveKV = 1.51;
        public final static double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public final static double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public final static double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public final static NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public final static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 1;
            public static final int angleMotorID = 2;
            public static final int driveMotorID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.48);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 11;
            public static final int angleMotorID = 12;
            public static final int driveMotorID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.64);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 21;
            public static final int angleMotorID = 22;
            public static final int driveMotorID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.84);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int canCoderID = 31;
            public static final int angleMotorID = 32;
            public static final int driveMotorID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.67);
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static double kPXController = 1;
        public static double kPYController = 1;
        public static double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}