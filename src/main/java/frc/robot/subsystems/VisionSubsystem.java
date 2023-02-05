package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

public class VisionSubsystem extends SubsystemBase {

    public static final String TABLE_NAME = "Vision";

    private final boolean report;
    private final DoubleArraySubscriber xArray;
    private final DoubleArraySubscriber yArray;
    private final DoubleArraySubscriber idArray;
    private double start;

    public VisionSubsystem(boolean report) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        this.xArray = table.getDoubleArrayTopic("target_x").subscribe(new double[0]);
        this.yArray = table.getDoubleArrayTopic("target_y").subscribe(new double[0]);
        this.idArray = table.getDoubleArrayTopic("target_id").subscribe(new double[0]);
        this.start = Timer.getFPGATimestamp();
        this.report = report;
    }

    public Translation3d getAprilTagPosition() {

        double [] x = xArray.get();
        double [] y = yArray.get();
        double [] id = idArray.get();

        if (x.length == 0) {
            return null;
        }

        if (x.length > 1) {
            // TODO how do we pick the "best" matched tag?
        }

        return new Translation3d(x[0], y[0], 0);
    }

    @Override
    public void periodic() {
        double since = Timer.getFPGATimestamp() - start;
        if (report && since > 3.0) {
            System.err.println("x = "+ Arrays.toString(xArray.get()));
            System.err.println("y = "+ Arrays.toString(yArray.get()));
            System.err.println("id = "+ Arrays.toString(idArray.get()));
            System.err.println();
            start = since;
        }
    }
}
