package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.SetRobotRelativeCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import frc.robot.commands.KyleAndChristopherCommand;
import frc.robot.commands.swerve.ZeroGyroCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.commands.swerve.AlignToWallCommand;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Button.kB;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;;

/**
 * All of the mapping of controls to commands for the production robot happens here.
 */
public class RobotControlMapping {

    public static final double DEADBAND = 0.1;

    /**
     * Creates a function that can be used to poll the joystick for speed controls.
     * Default implementation:
     *   - left Y is drive forward/backwards
     *   - left X is strafe left/right
     *   - right X is rotate left/right
     * 
     * Change this if you want to fiddle with how we drive the robot, or how
     * fast it responds to the controls.
     */
    public static Supplier<ChassisSpeeds> createSpeedSupplier(XboxController driverController) {
        return () -> {

            double vx = -driverController.getRawAxis(kLeftY.value) * SwerveConfig.maxSpeed; 
            double vy = -driverController.getRawAxis(kLeftX.value) * SwerveConfig.maxSpeed; 
            double vomega = -driverController.getRawAxis(kRightX.value)
            ;

            vx = MathUtil.applyDeadband(vx * Math.abs(vx), DEADBAND);
            vy = MathUtil.applyDeadband(vy * Math.abs(vy), DEADBAND);
            vomega = MathUtil.applyDeadband(vomega * Math.abs(vomega), DEADBAND);

            return new ChassisSpeeds(vx, vy, vomega);
        };
    }
    public static BooleanSupplier createHighSpeedSupplier(XboxController driverController) {
        return () -> !driverController.getRightBumper();
    }
    /**
     * Maps additional controls on the driver's joystick
     *    - X will trigger Kyle & Christopher's command
     *    - Y will trigger Ed's command
     *    - Start will zero the gyro again
     * 
     * Add additional button/axis mappings here.
     */
    public static void mapDriverControls(Robot robot, XboxController driverController) {
        trigger(driverController, kX, new KyleAndChristopherCommand(robot.swerveDrive));
        trigger(driverController, kY, SwerveFixedSpeedCommand.buildMultiStepProgram(robot.swerveDrive));
        trigger(driverController, kStart, new ZeroGyroCommand(robot.swerveDrive));
        trigger(driverController, kB, new AlignToWallCommand(robot));

        new JoystickButton(driverController, kLeftBumper.value)
                .onTrue(new SetRobotRelativeCommand(robot.swerveDrive, true))
                .onFalse(new SetRobotRelativeCommand(robot.swerveDrive, false));
    }

    /**
     * Maps controls on the special ops joystick.
     * 
     * Add additional button/axis mappings here.
     */
    public static void mapSpecialOpsControls(Robot robot, XboxController specialOpsController) {
        // trigger(specialOpsController, kY, new ExampleCommand(robot));
    }

    /**
     * Use this to make a specific button trigger a command
     */
    public static void trigger(XboxController controller, Button button, Command command) {
        new JoystickButton(controller, button.value).onTrue(command);
    }
}
