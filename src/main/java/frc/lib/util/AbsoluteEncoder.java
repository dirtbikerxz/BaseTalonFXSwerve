package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class AbsoluteEncoder {
    
    private final AnalogInput analogInput;

    private Rotation2d rotation2d = new Rotation2d();
    private Rotation2d home = new Rotation2d();

    public AbsoluteEncoder(int port) {

        analogInput = new AnalogInput(port);


    }

    public void homeEncoder() {

        this.home = this.rotation2d;

    }

    public Rotation2d getRotation() {
        double voltage = analogInput.getVoltage();
        double degrees = 360 * (voltage / Constants.Swerve.maxEncoderVoltage);
        return Rotation2d.fromDegrees(degrees);
    }

    /*
    public void getRotationPeriodic() {

        this.rotation2d = Rotation2d.fromDegrees(360 * (analogInput.getVoltage() / 5.0));
        System.out.println(rotation2d);

    }
    */
}