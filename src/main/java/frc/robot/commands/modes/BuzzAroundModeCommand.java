package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;

/**
 * Enters "buzz around mode":
 *   - robot relative is false
 *   - turbo mode is on
 *   - orbit mode is off
 *   - arm to travel position
 *
 * This command isn't over until all those actions have completed,
 * BUT we don't want to tie up the swerve drive while waiting for
 * the arm to reach its target position.
 */
public class BuzzAroundModeCommand extends SequentialCommandGroup {

    public BuzzAroundModeCommand(Robot robot) {

        // this is sort of cheating - we're not going to declare the
        // dependency on the swerve drive subsystem. all three of these
        // these will execute very quickly and then we'll move on
        addCommands(new InstantCommand(() -> robot.swerveDrive.setRobotRelative(false)));
        addCommands(new InstantCommand(() -> robot.swerveDrive.setTurboMode(true)));
        addCommands(new InstantCommand(() -> robot.swerveDrive.setOrbitMode(false)));

        // the real work - move the arm to the travel position.
        addCommands(new ArmPresetCommand(robot.arm, ArmPresetCommand.TRAVEL_POSITION));
    }
}
