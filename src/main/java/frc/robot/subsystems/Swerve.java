package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.SwerveModuleTalonNeo;
import frc.robot.SwerveModuleNeoNeo;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    /**
     * Default constructor uses SwerveModuleTalonNeo
     */
    public Swerve() {
        this(new SwerveModule[] {
            new SwerveModuleNeoNeo(Constants.Swerve.Mod0.constants),
            new SwerveModuleNeoNeo(Constants.Swerve.Mod1.constants),
            new SwerveModuleNeoNeo(Constants.Swerve.Mod2.constants),
            new SwerveModuleNeoNeo(Constants.Swerve.Mod3.constants)
        });
    }

    /**
     * Constructor that allows custom SwerveModules
     */
    public Swerve(SwerveModule[] modules) {
        this.mSwerveMods = modules;
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(desiredStates[i], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            states[i] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            positions[i] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute() {
        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            SmartDashboard.putNumber("Mod " + i + " CANcoder", mod.getRotation().getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + i + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}