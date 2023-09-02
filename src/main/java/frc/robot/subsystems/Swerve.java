package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotMap;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.DrivetrainConstants.Offsets;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    // public Pigeon2 gyro;

    public AHRS gyro;// = new AHRS(SPI.Port.kMXP);

    public Swerve() {
        // gyro = new Pigeon2(RobotMap.CAN.PIGEON, RobotMap.BUS.PIGEON);
        // gyro.getConfigurator().apply(new Pigeon2Configuration());
        // zeroGyro();
        gyro = new AHRS(SPI.Port.kMXP);

        //this can be compacted significantly, but this is what you have to do to make it work with our existing constants
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(
            0,
            new SwerveModuleConstants(
                RobotMap.CAN.FRONT_LEFT_DRIVE_MOTOR,
                RobotMap.CAN.FRONT_LEFT_AZIMUTH_MOTOR,
                RobotMap.CAN.FRONT_LEFT_CANCODER,
                RobotMap.BUS.DRIVE,
                RobotMap.BUS.AZIMUTH,
                RobotMap.BUS.CANCODER,
                Rotation2d.fromRotations(Offsets.FRONT_LEFT_STEER_OFFSET))),
        new SwerveModule(
            1,
            new SwerveModuleConstants(
                RobotMap.CAN.FRONT_RIGHT_DRIVE_MOTOR,
                RobotMap.CAN.FRONT_RIGHT_AZIMUTH_MOTOR,
                RobotMap.CAN.FRONT_RIGHT_CANCODER,
                RobotMap.BUS.DRIVE,
                RobotMap.BUS.AZIMUTH,
                RobotMap.BUS.CANCODER,
                Rotation2d.fromRotations(Offsets.FRONT_RIGHT_STEER_OFFSET))),
        new SwerveModule(2,
            new SwerveModuleConstants(
                RobotMap.CAN.BACK_LEFT_DRIVE_MOTOR,
                RobotMap.CAN.BACK_LEFT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_LEFT_CANCODER,
                RobotMap.BUS.DRIVE,
                RobotMap.BUS.AZIMUTH,
                RobotMap.BUS.CANCODER,
                Rotation2d.fromRotations(Offsets.BACK_LEFT_STEER_OFFSET))),
        new SwerveModule(3,
            new SwerveModuleConstants(
                RobotMap.CAN.BACK_RIGHT_DRIVE_MOTOR,
                RobotMap.CAN.BACK_RIGHT_AZIMUTH_MOTOR,
                RobotMap.CAN.BACK_RIGHT_CANCODER,
                RobotMap.BUS.DRIVE,
                RobotMap.BUS.AZIMUTH,
                RobotMap.BUS.CANCODER,
                Rotation2d.fromRotations(Offsets.BACK_RIGHT_STEER_OFFSET)))
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        // resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(DrivetrainConstants.SWERVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            DrivetrainConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void park() {
        //as of yet, non-functional
        mSwerveMods[0].setDesiredState(new SwerveModuleState(0d, DrivetrainConstants.FRONT_LEFT_RESTING_ANGLE), true);
        mSwerveMods[1].setDesiredState(new SwerveModuleState(0d, DrivetrainConstants.FRONT_RIGHT_RESTING_ANGLE), true);
        mSwerveMods[2].setDesiredState(new SwerveModuleState(0d, DrivetrainConstants.BACK_LEFT_RESTING_ANGLE), true);
        mSwerveMods[3].setDesiredState(new SwerveModuleState(0d, DrivetrainConstants.BACK_RIGHT_RESTING_ANGLE), true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        // gyro.setYaw(0);
        gyro.reset();
    }

    public Rotation2d getYaw() {
        // return gyro.getRotation2d(); //there used to be a invert gyro thingy here but I think phoenix 6 removes the need for that
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    // public void resetModulesToAbsolute(){
    //     for(SwerveModule mod : mSwerveMods){
    //         mod.resetToAbsolute();
    //     }
    // }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder (raw)", mod.getCanCoderRaw());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getAngleRaw());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}