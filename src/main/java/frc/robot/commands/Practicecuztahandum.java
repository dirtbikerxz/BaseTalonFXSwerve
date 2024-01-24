//Shooter code
public class RobotContainer {

    private final TalonSRX motorController;
    private final RunMotorCommand runMotorCommand;

    public RobotContainer() {
        motorController = new TalonSRX(1); // Replace 1 with your actual device ID
        runMotorCommand = new RunMotorCommand(motorController, 0.5); // Set the speed as needed
    }

    public void configureButtonBindings() {
        // Example: Trigger the motor command when a button is pressed
        new JoystickButton(driverController, Button.kA.value)
            .whenPressed(runMotorCommand);
    }
}
