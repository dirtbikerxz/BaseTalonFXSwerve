package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.RevSwerveModuleConstants;


/**
 * This file comes with command robot projects, and is intended to contain
 * configuration information.
 * I think it would be prudent if this file only contained CanIDs, because it
 * is useful to have all the ids for the whole robot in one place.
 * other configuration goes into subsystem specific configuration files,
 * to make sure this one isn't cluttered.
 */
public final class RevSwerveConstants 
{
    public static final double stickDeadband = 0.1;
    public static final double limelightOffset = 3;
  

    public static final class REV
    {
        public static final int pigeonID = 10;

       
    
    }
    public static final class Swerve {
        /* Module Specific Constants */
    /* Front Left Module - Module 0 */
        public static final class Mod0 { 

            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(293); //Rotation2d.fromDegrees(37.7);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(359);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(237);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
         /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221);
            public static final RevSwerveModuleConstants constants = 
                new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(21.73); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    }

     

    
    
    

   

}