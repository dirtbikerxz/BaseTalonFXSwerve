package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.TagApproaches;

public class Vision extends SubsystemBase {
    private TagApproaches tagApproches;
    public AprilTagFieldLayout FieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private static final Vision m_Vision = new Vision();

    public static Vision getInstance(){
        return m_Vision;
    }
    public Vision() {
        tagApproches = new TagApproaches();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    private void UpdateTargetData() {
        boolean targetAquired = LimelightHelpers.getTV("limelight-cybears");
        if (targetAquired) {
            int targetID = (int) LimelightHelpers.getFiducialID("limelight-cybears");
            SmartDashboard.putBoolean("driver/Target Found", targetAquired);
            SmartDashboard.putString("driver/TargetName", tagApproches.GameTargetName(targetID));

            Alliance targetAlliance = tagApproches.TagAlliance(targetID);
            if (MyAlliance() == tagApproches.TagAlliance(targetID)) {
                CalculateStearingValues(targetID);
            }
        }

    }

    private Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    private void CalculateStearingValues(int targetID) {

    }

    public void UpdatePoseEstimatorWithVisionBotPose(SwerveDrivePoseEstimator swervePoseEstimator) {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-cybears");

        // invalid LL data
        if (estimate.pose.getX() == 0.0) {
            return;
        }

        double a = estimate.pose.getX();
        double b = estimate.pose.getY() - 5.2;
        
        // sqrt(a^2+b^2)
        double poseDifference = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));

        LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight-cybears");

        if (llresults.targetingResults.valid) {
            if (poseDifference < 1.5){
                // swervePoseEstimator.setVisionMeasurementStdDevs(
                // VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
                swervePoseEstimator.addVisionMeasurement(estimate.pose,
                    Timer.getFPGATimestamp() - estimate.latency);
            }
        }
    }

    Pose2d GetLimelightPose() {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-cybears");
        if (estimate.pose.getX() == 0.0) {
            return new Pose2d();
        } else {
            return new Pose2d();
            //return estimate.pose;
        }
    }


}