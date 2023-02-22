package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.ExampleTrajectoryCommand;

/**
 * All of the mapping of controls to commands for the production robot happens here.
 */
public class RobotControlMapping {

    public static final double DEADBAND = 0.1;

    public static double clean(double input) {
        return MathUtil.applyDeadband(input * Math.abs(input), DEADBAND);
    }

    /**
     * Maps additional controls on the driver's joystick
     *    - X will trigger Kyle & Christopher's command
     *    - Y will trigger Ed's command
     *    - Start will zero the gyro again
     * 
     * Add additional button/axis mappings here.
     */
    public static void mapDriverControls(Robot robot, CommandXboxController driverController) {

        /*
         *   - left Y is drive forward/backwards
         *   - left X is strafe left/right
         *   - right X is rotate left/right
         */
        robot.swerveDrive.setDefaultCommand(new SwerveTeleopCommand(
                robot.swerveDrive,
                () -> clean(-driverController.getLeftY()),
                () -> clean(-driverController.getLeftX()),
                () -> clean(-driverController.getRightX())));

        driverController.y()
                .onTrue(SwerveFixedSpeedCommand.buildMultiStepProgram(robot.swerveDrive));

        driverController.start()
                .onTrue(SwerveCommands.zeroGyro(robot.swerveDrive));

        driverController.b()
                .onTrue(new ExampleTrajectoryCommand(robot.swerveDrive));

        driverController.x()
                .onTrue(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));

        driverController.leftStick()
                .onTrue(SwerveCommands.turnWheels(robot.swerveDrive, 90));

        driverController.leftBumper()
                .onTrue(SwerveCommands.setRobotRelative(robot.swerveDrive, true))
                .onFalse(SwerveCommands.setRobotRelative(robot.swerveDrive, false));

        driverController.rightBumper()
                .onTrue(SwerveCommands.setTurboMode(robot.swerveDrive, true))
                .onFalse(SwerveCommands.setTurboMode(robot.swerveDrive, false));

        driverController.rightTrigger(0.5)
                .onTrue(SwerveCommands.setOrbitMode(robot.swerveDrive, true))
                .onFalse(SwerveCommands.setOrbitMode(robot.swerveDrive, false));
    }
}
