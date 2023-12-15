package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.AlignedStates;
import frc.lib.math.Boundaries;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase{
    public SwerveDrivePoseEstimator sEstimator;
    public TimeInterpolatableBuffer<Double> turretYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public TimeInterpolatableBuffer<Double> gyroYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public Pose2d visionPose = new Pose2d();

    public PoseEstimator(){
        sEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, 
            new Rotation2d(), 
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            },
            new Pose2d(), 
            Constants.PoseEstimator.stateStdDevs, 
            Constants.PoseEstimator.visionStdDevs
        );
    }

    /** Check if this returns true before using {@link #updateVision()} 
     * @return If time buffers are !null */
    public boolean readyToUpdateVision(){
        return gyroYawBuffer.getSample(0).isPresent();
    }

    /** Update estimator with Swerve States and Gyro Yaw data.
     * Needs to be updated every loop. */
    public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions){
        sEstimator.update(gyroAngle, modulePositions);
        gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
    }

    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param LLlatency seconds */
    public void updateVision(Pose2d LLpose, double LLlatency){
        double timeStamp = Timer.getFPGATimestamp() - LLlatency;
        Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
        sEstimator.addVisionMeasurement(
            new Pose2d(LLpose.getX(), LLpose.getY(), gyro),
            timeStamp
        );
    }

    public Pose2d getEstimatedPosition(){
        return sEstimator.getEstimatedPosition();
    }

    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("robotX", getEstimatedPosition().getX());
        SmartDashboard.putNumber("robotY", getEstimatedPosition().getY());
        SmartDashboard.putNumber("robotHeading", getEstimatedPosition().getRotation().getRadians());

    }
}