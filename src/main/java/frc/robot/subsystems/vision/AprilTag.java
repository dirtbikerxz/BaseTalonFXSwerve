package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class AprilTag {

    public final double id;
    public final double imageX;
    public final double imageY;
    public final Transform3d pose;

    public AprilTag(double id, double imageX, double imageY, Transform3d pose) {
        this.id = id;
        this.imageX = imageX;
        this.imageY = imageY;
        this.pose = pose;
    }

    // the vision system uses "z" as the forward/reverse distance to the tag.
    // the robot will drive in its x dimension to close this distance. this
    // will hopefully limit confusion when writing the commands.
    public double getForwardReverseDistance() {
        return pose.getZ();
    }

    // the vision system uses "X" as the left/right position of the tag.
    // the robot will drive in its Y dimension to close this distance. this
    // will hopefully limit confusion when writing the commands.
    public double getLeftRightDistance() {
        return pose.getX();
    }

    public String toString() {
        return String.format("AprilTag(id=%d, imageX=%s, imageY=%s, pose=%s)", id, imageX, imageY, pose);
    }
}
