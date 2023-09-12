package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Pose4d extends Pose3d {
    double latency;

    public Pose4d(Translation3d translation, Rotation3d rotation, double latency) {
        super(translation, rotation);
        this.latency = latency;
    }

    public Pose4d(double x, double y, double z, double yaw, double pitch, double roll, double latency) {
        this(new Translation3d(x, y, z), new Rotation3d(yaw, pitch, roll), latency);
    }

    public double getLatency() {
        return latency;
    }

    public void setLatency(double latency) {
        this.latency = latency;
    }
}
