package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * This is a "skeleton" of a command that does nothing. To make a new command,
 * you can create a copy of this. Don't forget:
 *   - change "ExampleCommand" to something else (e.g. "MyAwesomeCommand")
 *   - make sure the file name corresponds to that (e.g. "MyAwesomeCommand.java")
 */
@SuppressWarnings("unused")
public class ExampleCommand extends CommandBase {

    private final Robot robot;

    public ExampleCommand(Robot robot) {

        this.robot = robot;

        // If you are going to use a subsystem in any of your methods, you
        // HAVE to uncomment the corresponding line below.
        // addRequirements(robot.swerveDrive);
        // addRequirements(robot.hand);
        // addRequirements(robot.arm);
        // addRequirements(robot.vision);
    }

    @Override
    public void initialize() {
        // REPLACE ME with real logic that actually does something
    }

    @Override
    public void execute() {
        // REPLACE ME with real logic that actually does something
    }

    @Override
    public boolean isFinished() {
        // REPLACE ME with real logic that returns true only when your command is finished running
        return true;
    }
}
