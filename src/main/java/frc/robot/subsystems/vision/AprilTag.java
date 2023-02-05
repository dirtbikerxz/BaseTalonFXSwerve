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

    public String toString() {
        return String.format("AprilTag(id=%d, imageX=%s, imageY=%s, pose=%s)", id, imageX, imageY, pose);
    }
}
