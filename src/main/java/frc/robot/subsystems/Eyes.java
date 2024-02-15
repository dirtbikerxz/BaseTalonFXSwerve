package frc.robot.subsystems;


import frc.robot.Constants;

import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;


public class Eyes extends SubsystemBase {

    public LimelightHelpers limelight;
    public double tx;
    public double ty;
    public double ta;
    public double tID;
  
    public Eyes() {}


    public void updateData() {

        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        tID = LimelightHelpers.getFiducialID("");

        SmartDashboard.putNumber("AprilTagX", tx);
        SmartDashboard.putNumber("AprilTagY", ty);
        SmartDashboard.putNumber("AprilTagA", ta);
        SmartDashboard.putNumber("AprilTagID", tID);

    }

    public double[] getDataPackage() {

        double[] data = {
            tx,
            ty,
            ta,
            tID
        };

        return data;
    }

    public Pose2d getRobotPose() {

        Pose2d pose;

        if(DriverStation.getAlliance().get() == Alliance.Red) {

            pose = LimelightHelpers.getBotPose2d_wpiRed("");

        } else {

            pose = LimelightHelpers.getBotPose2d_wpiBlue("");
        }

        return pose;

    }

    public Pose3d getTargetPose() {

        Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace("");

        return pose;
    }

    @Override
    public void periodic(){
        getTargetPose();
    }
}