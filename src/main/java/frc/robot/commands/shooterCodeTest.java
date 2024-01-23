

/* Shooter */
public class RunMotorCommand extends CommandBase {

    private final TalonSRX motorController;
    private final double speed;

    public RunMotorCommand(TalonSRX motorController, double speed) {
        this.motorController = motorController;
        this.speed = speed;
        addRequirements(motorController);
    }

    @Override
    public void execute() {
        motorController.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: Add cleanup code here
        motorController.set(0); // Stop the motor when the command ends
    }

    @Override
    public boolean isFinished() {
        return false; // This command never finishes on its own
    }
}
