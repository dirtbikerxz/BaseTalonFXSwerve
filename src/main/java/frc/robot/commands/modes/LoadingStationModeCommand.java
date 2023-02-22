package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;


/**
 * Enters "loading" mode for grabbing from the loading station
 *   - robot relative is false
 *   - turbo mode is off
 *   - orbit mode is off
 *   - align to wall (180)
 *   - align to april tag
 *   - arm to loading preset
 *
 * This command isn't over until all those actions have completed,
 * BUT we can do the arm thing and the driving things in parallel.
 */
public class LoadingStationModeCommand extends ParallelCommandGroup {

    public LoadingStationModeCommand(Robot robot) {
        addSwerveDriveCommands(robot);
        addArmCommands(robot);
    }

    private void addSwerveDriveCommands(Robot robot) {
        // TODO add some commands here - see BalanceModeCommand for an example of how to do this
    }

    private void addArmCommands(Robot robot) {
        // TODO add some commands here - see BalanceModeCommand for an example of how to do this
    }
}
