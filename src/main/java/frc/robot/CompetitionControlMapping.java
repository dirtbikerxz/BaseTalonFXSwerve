package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandCommands;
import frc.robot.commands.arm.ArmPresetCommand;
import frc.robot.commands.arm.ArmTeleopCommand;
import frc.robot.commands.modes.BuzzAroundModeCommand;
import frc.robot.commands.modes.DropOffModeCommand;
import frc.robot.commands.modes.LoadingStationModeCommand;
import frc.robot.commands.modes.PickupModeCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveTeleopCommand;

import static frc.robot.commands.arm.ArmPresetCommand.HIGH_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.LOW_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.MIDDLE_POSITION;
import static frc.robot.commands.arm.ArmPresetCommand.TRAVEL_POSITION;

public class CompetitionControlMapping {

    public static final double DEADBAND = 0.1;

    public static double clean(double input) {
        return MathUtil.applyDeadband(input * Math.abs(input), DEADBAND);
    }

    public static void mapDefaultCommands(Robot robot,
                                          CommandXboxController driverController,
                                          CommandXboxController specialOpsController) {

        /*
         * DEFAULT DRIVE
         *   - left Y is drive forward/backwards
         *   - left X is strafe left/right
         *   - right X is rotate left/right
         */
        robot.swerveDrive.setDefaultCommand(new SwerveTeleopCommand(
                robot.swerveDrive,
                () -> clean(-driverController.getLeftY()),
                () -> clean(-driverController.getLeftX()),
                () -> clean(-driverController.getRightX())));

        /*
         * DEFAULT TELEOP
         *   - left Y is rotate
         *   - right Y is extend
         */
        robot.arm.setDefaultCommand(new ArmTeleopCommand(
                robot.arm,
                () -> clean(specialOpsController.getLeftY()),
                () -> clean(specialOpsController.getRightY())));
    }

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
        controller.b().onTrue(new LoadingStationModeCommand(robot));
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
