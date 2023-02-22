package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.modes.BuzzAroundModeCommand;
import frc.robot.commands.modes.DropOffModeCommand;
import frc.robot.commands.modes.PickupModeCommand;
import frc.robot.commands.swerve.SwerveCommands;

import static frc.robot.commands.arm.ArmPresetCommand.HIGH_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.LOW_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.MIDDLE_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.TRAVEL_POSITION;

public class CompetitionControlMapping {

    public static void mapDriverJoystick(Robot robot, CommandXboxController controller) {

        controller.start()
                .onTrue(SwerveCommands.zeroGyro(robot.swerveDrive));

        controller.rightTrigger(0.5)
                .onTrue(SwerveCommands.setOrbitMode(robot.swerveDrive, true))
                .onTrue(SwerveCommands.setOrbitMode(robot.swerveDrive, false));

        controller.x()
                .onTrue(HandCommands.grab(robot.hand));

        controller.y()
                .onTrue(HandCommands.release(robot.hand));

        controller.rightBumper().onTrue(new BuzzAroundModeCommand(robot));
        controller.leftBumper().onTrue(new PickupModeCommand(robot));

        // ??? what button should we map to the loading command?
        // controller.???().onTrue(new LoadingStationModeCommand(robot));
    }

    public static void mapSpecialOpsJoystick(Robot robot, CommandXboxController specialOpsController) {
        
        specialOpsController.pov(0).onTrue(new ArmPresetCommand(robot.arm, HIGH_POSITION));
        specialOpsController.pov(90).onTrue(new ArmPresetCommand(robot.arm, MIDDLE_POSITION));
        specialOpsController.pov(180).onTrue(new ArmPresetCommand(robot.arm, LOW_POSITION));
        specialOpsController.pov(270).onTrue(new ArmPresetCommand(robot.arm, TRAVEL_POSITION));

        specialOpsController.a()
                .onTrue(new DropOffModeCommand(robot));

        specialOpsController.x()
                .onTrue(HandCommands.grab(robot.hand));

        specialOpsController.y()
                 .onTrue(new HandCommands().release(robot.hand));

        specialOpsController.leftBumper()
                .onTrue(SwerveCommands.hopLeft(robot.swerveDrive, 22.0));

        specialOpsController.rightBumper()
                .onTrue(SwerveCommands.hopRight(robot.swerveDrive, 22.0));
    }
    
}
