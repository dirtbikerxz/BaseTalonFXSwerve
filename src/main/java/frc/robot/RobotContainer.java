package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
    private final Joystick driver = new Joystick(Constants.DRIVER_PORT);
    private final Joystick operator = new Joystick(Constants.OPERATOR_PORT);

    /* Drive Controls */
    private final int driverLeftY = XboxController.Axis.kLeftY.value;
    private final int driverLeftX = XboxController.Axis.kLeftX.value;
    private final int driverRightX = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverLB = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRB = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);
    private final JoystickButton driverRStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final POVButton driverDpadUp = new POVButton(driver, 0);
    private final POVButton driverDpadRight = new POVButton(driver, 90);
    private final POVButton driverDpadDown = new POVButton(driver, 180);
    private final POVButton driverDpadLeft = new POVButton(driver, 270);

    /* Operator Buttons */
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorLB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final POVButton operatorDpadUp = new POVButton(operator, 0);
    private final POVButton operatorDpadRight = new POVButton(operator, 90);
    private final POVButton operatorDpadDown = new POVButton(operator, 180);
    private final POVButton operatorDpadLeft = new POVButton(operator, 270);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final LEDs leds = new LEDs();
    private final Intake intake = new Intake();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //compressor.disable();
        //arm.setDefaultCommand(new MoveArmManual(arm, driver));
        leds.setDefaultCommand(new IdleLEDS(leds));
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

        swerveHandler();
        intakeHandler();
        lightHandler();
        elevatorHandler();
        armHandler();

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


    public void intakeHandler() {

        driverLB.whileTrue(new OpenIntake(intake));
        driverRB.whileTrue(new CloseIntake(intake));

        driverA.whileTrue(new RunIntake(intake));
        driverX.whileTrue(new ReverseIntake(intake));

    }

    public void lightHandler() {

        operatorLB.whileTrue(new PurpleLED(leds));
        operatorRB.whileTrue(new YellowLED(leds));

    }

    public void swerveHandler() {

        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverB.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
    }

    public void idleAnimation() {
        new InstantCommand(() -> new IdleLEDS(leds));
    }

    public void elevatorHandler() {

        //positions
        operatorX.whileTrue(new PositionElevator(elevator, Constants.MID_LEVEL));
        operatorY.whileTrue(new PositionElevator(elevator, Constants.HIGH_LEVEL));
        operatorA.whileTrue(new PositionElevator(elevator, Constants.LOW_LEVEL));

        //manual
        operatorDpadUp.whileTrue(new ManualUp(elevator));
        operatorDpadDown.whileTrue(new ManualDown(elevator));

    }

    public void armHandler() {

        // arm.setDefaultCommand(new SetArmPosition(arm, Constants.ARM_STOW_POSITION));
        
        operatorDpadLeft.whileTrue(new MoveArmUp(arm));
        operatorDpadRight.whileTrue(new MoveArmDown(arm));

        operatorLStick.whileTrue(new SetArmPosition(arm, Constants.ARM_LOW_POSITION));
        operatorRStick.whileTrue(new SetArmPosition(arm, Constants.ARM_MID_POSITION));
        operatorB.whileTrue(new SetArmPosition(arm, Constants.ARM_HIGH_POSITION));

    }
}
