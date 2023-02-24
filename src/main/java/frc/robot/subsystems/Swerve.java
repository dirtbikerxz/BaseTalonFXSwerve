package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
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
        gyro.setYaw(Constants.GRYO_OFFSET);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getRoll());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        SmartDashboard.putNumber("RobotCoordinatesX",swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("RobotCoordinatesY",swerveOdometry.getPoseMeters().getY());

        for(SwerveModule mod : mSwerveMods){
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            //SmartDashboard.putNumber("Mod " + mod.moduleNumber + "target angle", mod.getAngle().getDegrees());
        }
    }


    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getInitialHolonomicPose());
                SmartDashboard.putNumber ("InitialXPos", traj.getInitialHolonomicPose().getX());
                SmartDashboard.putNumber ("InitialYPos", traj.getInitialHolonomicPose().getY());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(5, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                this // Requires this drive subsystem
            )
        );
    }

    public Command LockWheels() {
        Translation2d stop = new Translation2d(0.0, 0.0);
        return new InstantCommand(() -> drive(stop, 90.0, true, false));
    }

    public void lockWheels() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(90))
        });
    }
}