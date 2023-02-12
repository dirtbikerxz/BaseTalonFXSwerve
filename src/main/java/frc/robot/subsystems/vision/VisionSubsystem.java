package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTag;

public class VisionSubsystem extends SubsystemBase {

    public static final String TABLE_NAME = "Vision";

    private final DoubleArraySubscriber transX;
    private final DoubleArraySubscriber transY;
    private final DoubleArraySubscriber transZ;
    private final DoubleArraySubscriber rotX;
    private final DoubleArraySubscriber rotY;
    private final DoubleArraySubscriber rotZ;
    private final DoubleArraySubscriber idArray;

    public VisionSubsystem(boolean report) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        this.transX = table.getDoubleArrayTopic("target_pose_tx").subscribe(new double[0]);
        this.transY = table.getDoubleArrayTopic("target_pose_ty").subscribe(new double[0]);
        this.transZ = table.getDoubleArrayTopic("target_pose_tz").subscribe(new double[0]);
        this.rotX = table.getDoubleArrayTopic("target_pose_rx").subscribe(new double[0]);
        this.rotY = table.getDoubleArrayTopic("target_pose_ry").subscribe(new double[0]);
        this.rotZ = table.getDoubleArrayTopic("target_pose_rz").subscribe(new double[0]);
        this.idArray = table.getDoubleArrayTopic("robot_id").subscribe(new double[0]);
    }

    public AprilTag getAprilTag() {

        double [] ptx = transX.get();
        double [] pty = transY.get();
        double [] ptz = transZ.get();
        double [] prx = rotX.get();
        double [] pry = rotY.get();
        double [] prz = rotZ.get();
        double [] id = idArray.get();

        if (!hasAtLeastOne(ptx, pty, ptz, prx, pry, prz, id)) {
            System.err.println("no tag in view");
            return null;
        }

        Rotation3d r = new Rotation3d(prx[0], pry[0], prz[0]);
        Translation3d t = new Translation3d(ptx[0], pty[0], ptz[0]);
        return new AprilTag(id[0], new Transform3d(t, r));
    }

    private boolean hasAtLeastOne(double [] ...arrays) {
        for (double [] array : arrays) {
            if (array == null || array.length < 1) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void periodic() {
    }
}
