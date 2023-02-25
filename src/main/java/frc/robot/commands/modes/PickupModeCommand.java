package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.AlignToWallCommand;

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

        turningOffSwerveDriveModes(robot);
        new ArmPresetCommand(robot.arm, ArmPresetCommand.PICKUP_POSITION);
        
    }

    public void turningOffSwerveDriveModes(Robot robot){

        new InstantCommand(() -> robot.swerveDrive.setTurboMode(false));
        new InstantCommand(() -> robot.swerveDrive.setOrbitMode(false));
        new InstantCommand(() -> robot.swerveDrive.setRobotRelative(false));

    }

}

    