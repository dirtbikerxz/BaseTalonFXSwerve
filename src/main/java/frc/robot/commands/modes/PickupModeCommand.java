package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**
 * Enters pickup mode for picking up from the floor:
 *   - robot relative is true
 *   - turbo mode is off
 *   - orbit mode is off
 *   - arm to pickup preset
 *
 * This command isn't over until all those actions have completed.
 */
public class PickupModeCommand extends CommandBase {

    public PickupModeCommand(Robot robot) {
        // TODO add some commands here - see BalanceModeCommand for an example of how to do this
    }
}
