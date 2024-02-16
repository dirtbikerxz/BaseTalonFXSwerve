package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Eyes eyes;
    public StructPublisher<Pose2d> publisher;
    public StructArrayPublisher<SwerveModuleState> swerveKinematicsPublisher;
    public StructPublisher<Pose2d> estimatedRobotPosePublisher;

    public SwerveDrivePoseEstimator m_poseEstimator;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        eyes = new Eyes();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        publisher = NetworkTableInstance.getDefault().getStructTopic("/MyPose", Pose2d.struct).publish();

        swerveKinematicsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveModuleStates", SwerveModuleState.struct).publish();

        estimatedRobotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedRobotPose", Pose2d.struct).publish();
        
        m_poseEstimator =
        new SwerveDrivePoseEstimator(
           Constants.Swerve.swerveKinematics,
           gyro.getRotation2d(),
           getModulePositions(),
           new Pose2d(),
           VecBuilder.fill(0.1, 0.1, 0.1),
           VecBuilder.fill(1.5, 1.5, 1.5)
        );

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

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Pose3d getTargetPose() {

        Pose3d pose;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            pose = new Pose3d(Constants.Positions.speakerBlueX, Constants.Positions.speakerBlueY, 0, new Rotation3d(0,0,Constants.Positions.speakerBlueR));

        } else {

            pose = new Pose3d(Constants.Positions.speakerRedX, Constants.Positions.speakerRedY, 0, new Rotation3d(0,0,Constants.Positions.speakerRedR));

        }
        
        return pose;

    }

    public double getDistanceFromTarget() {

        double distance;

        if(DriverStation.getAlliance().get() == Alliance.Blue) {

            distance = Math.sqrt(Math.pow((Constants.Positions.speakerBlueX - m_poseEstimator.getEstimatedPosition().getX()), 2) - Math.pow((Constants.Positions.speakerBlueY - m_poseEstimator.getEstimatedPosition().getY()), 2));

        } else {

            distance = Math.sqrt(Math.pow((Constants.Positions.speakerRedX - m_poseEstimator.getEstimatedPosition().getX()), 2) - Math.pow((Constants.Positions.speakerRedY - m_poseEstimator.getEstimatedPosition().getY()), 2));

        }

        return distance;
    }

    public double getTargetRotation() {

        Pose2d robotPose = m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));

        if (robotX > targetX) {

            angle = angle + 180;

        }

        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber(" inverted angle", -angle);

        return -angle;
    }

    @Override
    public void periodic(){

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        //setPose(eyes.getRobotPose());

        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        if (LimelightHelpers.getTV("") == true) {
            m_poseEstimator.addVisionMeasurement(eyes.getRobotPose(), Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("")/1000.0) - (LimelightHelpers.getLatency_Capture("")/1000.0));
        }

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Robot X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("gyro angle", getGyroYaw().getDegrees());

        SmartDashboard.putNumber("Pose estimator rotations", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("robot X", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("robot Y", m_poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("target X", getTargetPose().getX());
        SmartDashboard.putNumber("target Y", getTargetPose().getY());

        publisher.set(getPose());
        swerveKinematicsPublisher.set(getModuleStates());
        estimatedRobotPosePublisher.set(m_poseEstimator.getEstimatedPosition());
    }
}