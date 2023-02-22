package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

/**
 * Enters "drop off" mode for depositing to the grid:
 *   - robot relative is false
 *   - turbo mode is off
 *   - orbit mode is off
 *   - align to wall (0)
 *   - align to april tag
 * This command isn't over until all those actions have completed.
 */
public class DropOffModeCommand extends SequentialCommandGroup {

    public DropOffModeCommand(Robot robot) {
        // TODO add some commands here - see BalanceModeCommand for an example of how to do this
    }
}
