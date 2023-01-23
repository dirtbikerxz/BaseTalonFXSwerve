package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.EdsCommand;
import frc.robot.commands.KyleAndChristopherCommand;
import frc.robot.commands.ZeroGyroCommand;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;;

public class RobotControlMapping {

    public static final double DEADBAND = 0.1;

    /**
     * Creates a function that can be used to poll the joystick for speed controls.
     * Default implementation:
     *   - left Y is drive forward/backwards
     *   - left X is strafe left/right
     *   - right X is rotate left/right
     */
    public static Supplier<ChassisSpeeds> createSpeedSupplier(XboxController driverController) {
        return () -> {

            double vx = -driverController.getRawAxis(kLeftY.value); 
            double vy = -driverController.getRawAxis(kLeftX.value); 
            double vomega = -driverController.getRawAxis(kRightX.value);

            vx = MathUtil.applyDeadband(vx * Math.abs(vx), DEADBAND);
            vy = MathUtil.applyDeadband(vy * Math.abs(vy), DEADBAND);
            vomega = MathUtil.applyDeadband(vomega * Math.abs(vomega), DEADBAND);

            return new ChassisSpeeds(vx, vy, vomega);
        };
    }

    /**
     * Creates a function that can be used to poll the joystick and determine
     * whether we're driving in field-relative mode. Default implementation:
     *   - hold the left bumper to be in robot-relative mode
     */
    public static BooleanSupplier createFieldRelativeSupplier(XboxController driverController) {
        return () -> !driverController.getLeftBumper();
    }

    /**
     * Maps additional controls on the driver's joystick
     *    - X will trigger Kyle & Christopher's command
     *    - Y will trigger Ed's command
     *    - Start will zero the gyro again
     */
    public static void mapDriverControls(Robot robot, XboxController driverController) {
        trigger(driverController, kX, new KyleAndChristopherCommand(robot.swerveDrive));
        trigger(driverController, kY, EdsCommand.buildMultiStepProgram(robot.swerveDrive));
        trigger(driverController, kStart, new ZeroGyroCommand(robot.swerveDrive));
    }

    public static void mapSpecialOpsControls(Robot robot, XboxController specialOpsController) {

    }

    public static void trigger(XboxController controller, Button button, Command command) {
        new JoystickButton(controller, button.value).onTrue(command);
    }
}
