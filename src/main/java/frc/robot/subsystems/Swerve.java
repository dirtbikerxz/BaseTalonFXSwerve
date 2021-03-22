package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public Swerve() {
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(1, 
                Constants.Swerve.Mod1.angleOffset, Constants.Swerve.Mod1.angleID, 
                Constants.Swerve.Mod1.canCoderID, Constants.Swerve.Mod1.driveID,
                Constants.Swerve.Mod1.driveInvert, Constants.Swerve.Mod1.angleInvert,
                Robot.ctreConfigs.mod1AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig,
                Robot.ctreConfigs.swerveCanCoderConfig),
            new SwerveModule(2, 
                Constants.Swerve.Mod2.angleOffset, Constants.Swerve.Mod2.angleID, 
                Constants.Swerve.Mod2.canCoderID, Constants.Swerve.Mod2.driveID, 
                Constants.Swerve.Mod2.driveInvert, Constants.Swerve.Mod2.angleInvert, 
                Robot.ctreConfigs.mod2AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig,
                Robot.ctreConfigs.swerveCanCoderConfig),
            new SwerveModule(3, 
                Constants.Swerve.Mod3.angleOffset, Constants.Swerve.Mod3.angleID, 
                Constants.Swerve.Mod3.canCoderID, Constants.Swerve.Mod3.driveID, 
                Constants.Swerve.Mod3.driveInvert, Constants.Swerve.Mod3.angleInvert, 
                Robot.ctreConfigs.mod3AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig,
                Robot.ctreConfigs.swerveCanCoderConfig),
            new SwerveModule(4, 
                Constants.Swerve.Mod4.angleOffset, Constants.Swerve.Mod4.angleID, 
                Constants.Swerve.Mod4.canCoderID, Constants.Swerve.Mod4.driveID, 
                Constants.Swerve.Mod4.driveInvert, Constants.Swerve.Mod4.angleInvert, 
                Robot.ctreConfigs.mod4AngleFXConfig, Robot.ctreConfigs.swerveDriveFXConfig,
                Robot.ctreConfigs.swerveCanCoderConfig)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
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
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber - 1], isOpenLoop);
        }
    }    

    //set module states for auto
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber - 1], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber-1] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    public void setGyro(double input){
        gyro.setYaw(input);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        if (Constants.Swerve.invertGyro){
            return (Rotation2d.fromDegrees(360 - ypr[0]));
        }
        else{
            return Rotation2d.fromDegrees(ypr[0]);
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());        
    }
}