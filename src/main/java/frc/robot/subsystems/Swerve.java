package frc.robot.subsystems;

import frc.lib.doubleNeoConstants;
import frc.robot.interfaces.SwerveModule;
import frc.robot.swervemodules.SwerveModuleDoubleNeo;
import frc.robot.swervemodules.SwerveModuleKrakenFalcon;
import frc.robot.swervemodules.SwerveModuleDoubleTalon;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swervemodules.SwerveModuleKrakenFalcon;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    /**
     * Default constructor uses SwerveModuleTalonNeo
     */
    public Swerve() {
        this(new SwerveModule[]{
                new SwerveModuleKrakenFalcon(doubleNeoConstants.Swerve.Mod0.constants, 0),
                new SwerveModuleKrakenFalcon(doubleNeoConstants.Swerve.Mod1.constants, 1),
                new SwerveModuleKrakenFalcon(doubleNeoConstants.Swerve.Mod2.constants, 2),
                new SwerveModuleKrakenFalcon(doubleNeoConstants.Swerve.Mod3.constants, 3)
        });
    }

    /**
     * Constructor that allows custom SwerveModules
     */
    public Swerve(SwerveModule[] modules) {
        this.mSwerveMods = modules;
        gyro = new Pigeon2(doubleNeoConstants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        swerveOdometry = new SwerveDriveOdometry(doubleNeoConstants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading()
        ) : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);

        SwerveModuleState[] swerveModuleStates = doubleNeoConstants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, doubleNeoConstants.Swerve.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(swerveModuleStates[i], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, doubleNeoConstants.Swerve.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.setDesiredState(desiredStates[i], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            states[i] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
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

    public Rotation2d getHeading() {
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
        for (int i = 0; i < mSwerveMods.length; i++) {
            SwerveModule mod = mSwerveMods[i];
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.dashboardPeriodic();
        }
    }
}