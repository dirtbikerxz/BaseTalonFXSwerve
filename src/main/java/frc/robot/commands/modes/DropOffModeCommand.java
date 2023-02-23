package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
        addCommands(new InstantCommand(() -> robot.swerveDrive.setOrbitMode(false)));
        addCommands(new InstantCommand(() -> robot.swerveDrive.setTurboMode(false)));
        addCommands(new InstantCommand(() -> robot.swerveDrive.setRobotRelative(false)));

        addCommands(new AlignToWallCommand(robot, 180));
        //the wanted direction depends on the postition of the robot?

        addCommands(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));
    }
}
