// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
import frc.robot.subsystems.swerve.falcon.CTRESwerveModule;

/** Add your docs here. */
public class CTRESwerveConstants {
    public static final class Swerve {
        public static final int pigeonID = 1;
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final String canBus = "rio"; //"rio" (by default), or the name of your canivore
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final CTRESwerveModule constants = 
                new CTRESwerveModule(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final String canBus = "rio"; //"rio" (by default), or the name of your canivore
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final CTRESwerveModule constants = 
                new CTRESwerveModule(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final String canBus = "rio"; //"rio" (by default), or the name of your canivore
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final CTRESwerveModule constants = 
                new CTRESwerveModule(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final String canBus = "rio"; //"rio" (by default), or the name of your canivore
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
            public static final CTRESwerveModule constants = 
                new CTRESwerveModule(driveMotorID, angleMotorID, canCoderID, angleOffset, canBus);
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
