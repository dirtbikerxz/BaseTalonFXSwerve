package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int driverLeftY = XboxController.Axis.kLeftY.value;
    private final int driverLeftX = XboxController.Axis.kLeftX.value;
    private final int driverRightX = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);

    private final JoystickButton driverR = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverL = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton driverDPadDown = new POVButton(driver, 180);
    

    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final LEDs leds = new LEDs();
    public final Elevator elevator = new Elevator();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(driverLeftY), 
                () -> -driver.getRawAxis(driverLeftX), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverStart.getAsBoolean()
            )
            // new DriveForward(s_Swerve)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverX.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        //purplelights.onTrue(new InstantCommand(( new RainbowLED(leds))));

        driverR.onTrue(new PurpleLED(leds));
        driverL.onTrue(new YellowLED(leds));

        // driverA.whileTrue(new ExtendElevator(elevator));
        // driverB.whileTrue(new RetractElevator(elevator));
        driverB.whileTrue(new PositionElevator(elevator, 15));
        driverA.whileTrue(new PositionElevator(elevator, 30));
        driverDPadDown.whileTrue(new PositionElevator(elevator, Constants.REVERSE_ELEVATOR_LIMIT));
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public void resetAbsolute() {
        s_Swerve.resetModulesToAbsolute();
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return s_Swerve.followTrajectoryCommand(traj, isFirstPath);
    }
}
