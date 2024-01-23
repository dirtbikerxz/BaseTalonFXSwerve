import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

    private TalonSRX motorController;
    private Joystick joystick;

    @Override
    public void robotInit() {
        // Initialize the motor controller with the appropriate device ID
        motorController = new TalonSRX(1); // Replace 1 with your actual device ID
        joystick = new Joystick(0); // Replace 0 with your actual joystick port
    }

    @Override
    public void teleopPeriodic() {
        // Control the motor based on joystick input during teleop (driver control)
        double speed = joystick.getRawAxis(1); // Get the Y-axis value of the joystick
        motorController.set(speed);
    }
}
