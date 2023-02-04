package frc.robot.subsystems;

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
    private final StringArraySubscriber idArray;
    private double start;

    public VisionSubsystem(boolean report) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        this.xArray = table.getDoubleArrayTopic("target_x").subscribe(new double[0]);
        this.yArray = table.getDoubleArrayTopic("target_y").subscribe(new double[0]);
        this.idArray = table.getStringArrayTopic("target_id").subscribe(new String[0]);
        this.start = Timer.getFPGATimestamp();
        this.report = report;
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
