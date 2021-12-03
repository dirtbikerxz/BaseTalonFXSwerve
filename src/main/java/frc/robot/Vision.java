package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    NetworkTableEntry cameraSelection;
    NetworkTableEntry limelight;
    
    final double x1 = -0.0000000025291;
    final double x2 = 0.0000334240538;
    final double x3 = -0.1545379987062;
    final double b = 315.5170993015826;

    double disX = 0, disY = 0, tx = 0, ty = 0, ta = 0, tv = 0;
    boolean targetFound = false;

    public double update(){  
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);

        if (tv < 1.0) {
                targetFound = false;
                disX = 0;
                disY = 0;
            } else {
                targetFound = true;
                disX = tx;
                disY = ty;
            }

        // m_turretMotor.set(s);
        return disX / 125;
    }

    public double getAimValue(){  
        // System.out.println(update());
        return (update());
        // return 0;
    }
}
