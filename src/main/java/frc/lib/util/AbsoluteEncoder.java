package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class AbsoluteEncoder {
    
    private final AnalogInput analogInput;

    private Rotation2d rotation2d = new Rotation2d();
    private Rotation2d home = new Rotation2d();

    public AbsoluteEncoder(int port) {

        analogInput = new AnalogInput(port);


    }

    public void homeEncoder() {

        home = rotation2d;

    }

    public Rotation2d getRotation() {

        return home.rotateBy(rotation2d);

    }

    public void getRotationPeriodic() {

        Rotation2d rotation2d = Rotation2d.fromDegrees(360 * (analogInput.getVoltage() / 5.0));

    }

}
