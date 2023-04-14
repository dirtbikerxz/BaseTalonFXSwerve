package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double MAX_ENCODER_VOLTAGE = 4.926;

    /* WRIST Constants */
    // TODO: Change later when we have actual robot
    public static final int WRIST_MOTOR_ID = 20;
    public static final int WRIST_ENCODER_ID = 1;
    public static final double WRIST_FORWARD_LIMIT = 40;
    public static final double WRIST_REVERSE_LIMIT = -90;
    public static final double WRIST_GEAR_RATIO = 87.27;
    public static final double WRIST_MOTOR_ROT_TO_DEG = 360.0 / WRIST_GEAR_RATIO;
    public static final double MANUAL_WRIST_SPEED = 1.0;
    public static final double WRIST_GROUND_POSITION = -95; //set value

    // positions
    public static final double WRIST_CONE_STOW_POSITION = 58.58;
    public static final double WRIST_CONE_LOW_POSITION = -25.8;
    public static final double WRIST_CONE_MID_POSITION = -30;
    public static final double WRIST_CONE_HIGH_POSITION = 50;
    public static final double WRIST_CONE_SINGLE_POSITION = 0;
    public static final double WRIST_CONE_DOUBLE_POSITION = 0;

    public static final double WRIST_CUBE_STOW_POSITION = 58.58;
    public static final double WRIST_CUBE_LOW_POSITION = -25.8;
    public static final double WRIST_CUBE_MID_POSITION = 9.5;
    public static final double WRIST_CUBE_HIGH_POSITION = 0;
    public static final double WRIST_CUBE_SINGLE_POSITION = 0;
    public static final double WRIST_CUBE_DOUBLE_POSITION = 0;

    public static final double WRIST_DEFAULT_STOW_POSITION = 58.58;
    public static final double WRIST_DEFAULT_LOW_POSITION = -25.8;
    public static final double WRIST_DEFAULT_MID_POSITION = 0;
    public static final double WRIST_DEFAULT_HIGH_POSITION = 50;
    public static final double WRIST_DEFAULT_SINGLE_POSITION = 0;
    public static final double WRIST_DEFAULT_DOUBLE_POSITION = 0;
    

    public static final double WRIST_TOLERANCE = 20.0;

    //Feedforward
    public static final double WRIST_G = 0.2;
    public static final double WRIST_V = 0.0;
    public static final double WRIST_A = 0.0;
    public static final double WRIST_S = 0.0;

    //PID
    public static final double WRIST_P = 0.3;
    public static final double WRIST_I = 0.0;
    public static final double WRIST_D = 0.0;

    public static final double AUTO_X_P = 4;
    public static final double AUTO_X_I = 0;
    public static final double AUTO_X_D = 0;

    public static final double AUTO_Y_P = 4;
    public static final double AUTO_Y_I = 0;
    public static final double AUTO_Y_D = 0;

    public static final double AUTO_R_P = 6;
    public static final double AUTO_R_I = 0;
    public static final double AUTO_R_D = 0;
    
    
    //TODO: Set offset
    public static final double WRIST_ENCODER_OFFSET = -1.3;
    public static final double TARGET_WRIST_ANGLE = 0;

    public static final double INTAKE_CONE_SPEED = -0.5;
    public static final double INTAKE_CUBE_SPEED = 0.5;

    public static int INTAKE_MOTOR_1_ID = 12;
    public static int INTAKE_MOTOR_2_ID = 11;
    
    public static final int PNEUMATIC_HUB_ID = 15;
    public static final int PNEUMATIC_FORWARD_CHANNEL = 14;
    public static final int PNEUMATIC_REVERSE_CHANEL = 15;

    public static final int CANDLE_PORT = 9;
    public static final double WRIST_ENCODER_RATIO = 1.4545;

    /* Elevator Constants */
    public static final int FORWARD_ELEVATOR_LIMIT = 26;
    public static final float REVERSE_ELEVATOR_LIMIT = (float) -0.5;
    public static final int ELEVATOR_MOTOR_ID = 16;
    public static final double ELEVATOR_GEAR_RATIO = 9.0;
    public static final double ELEVATOR_SPROCKET_DIAMETER = 1.751;
    public static final double ELEVATOR_ROTATIONS_TO_IN = 1.0/ELEVATOR_GEAR_RATIO * ELEVATOR_SPROCKET_DIAMETER * Math.PI;
    public static final double MANUAL_ELEVATOR_SPEED = 0.50;

    public static final double ELEVATOR_P = 0.75;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.0;
    public static final double ELEVATOR_G = 0.32;

    public static final double ELEVATOR_CONE_HIGH_LEVEL = 26;
    public static final double ELEVATOR_CONE_MID_LEVEL = ELEVATOR_CONE_HIGH_LEVEL;
    public static final double ELEVATOR_CONE_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_CONE_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_CONE_STOW_LEVEL = 0;
    public static final double ELEVATOR_CONE_SINGLE_POSITION = 4;
    public static final double ELEVATOR_CONE_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_CUBE_HIGH_LEVEL = 26;
    public static final double ELEVATOR_CUBE_MID_LEVEL = 14.45;
    public static final double ELEVATOR_CUBE_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_CUBE_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_CUBE_STOW_LEVEL = 0;
    public static final double ELEVATOR_CUBE_SINGLE_POSITION = 4;
    public static final double ELEVATOR_CUBE_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_DEFUALT_HIGH_LEVEL = 26;
    public static final double ELEVATOR_DEFUALT_MID_LEVEL = 8.5;
    public static final double ELEVATOR_DEFUALT_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_DEFUALT_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_DEFUALT_STOW_LEVEL = 0;
    public static final double ELEVATOR_DEFUALT_SINGLE_POSITION = 4;
    public static final double ELEVATOR_DEFUALT_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_TOLERANCE = 5;

    public static final int GRYO_OFFSET = 0;

    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    // auto balancing constants
    public static final double AUTO_BALANCE_P = 0.03;
    public static final double AUTO_BALANCE_I = 0.00;
    public static final double AUTO_BALANCE_D = 0.01;
    public static final double RED_BALANCE_LEVEL = -2.4;
    public static final double BLUE_BALANCE_LEVEL = -2.4;
    public static final double BALANCE_LEVEL_DEADZONE = 5.0;
    public static final double WHEEL_LOCK_TIME = 0.1;

     /* PID Rotation */ 
     public static final double ROTATE_KP = 0.01; //0.0222
     public static final double ROTATE_KI = 0.0;
     public static final double ROTATE_KD = 0.0;
     public static final double ROTATE_VELOCITY = 200.0;
     public static final double ROTATE_ACCELERATION = 400.0;
     public static final double ROTATE_TO_SCORE_TARGET_ANGLE = 180.0;
     public static final double ROTATE_TO_LOAD_TARGET_ANGLE = 0.0;
     public static final double AUTO_ROTATE_DEADBAND = 5.0;

     /* slow mode */
     public static final double SLOW_MODE_PERCENT_TRANSLATION = 0.5;
     public static final double SLOW_MODE_PERCENT_STRAFE = 0.5;
     public static final double SLOW_MODE_PERCENT_ROTATION = 0.5;

     /* auto */
    public static final double AUTO_VEL = 2.0;
    public static final double AUTO_ACC = 2.0;

    public static final class Swerve {
        public static final int pigeonID = 8;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: 2023 More robot 18.75in
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: 2023 More robot 18.75in
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

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.0001;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.9; // Free speed on blocks
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // This is definitely not the max, but no need to change it
        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int encoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(20.55); //189.22,20.30
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int encoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(274.412); //89.22,265
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int encoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.62); //18.80,192.48
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int encoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141.92); //307.00,315.08
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}