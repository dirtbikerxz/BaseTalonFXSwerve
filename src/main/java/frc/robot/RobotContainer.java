package frc.robot;

import java.util.function.DoubleSupplier;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.GameAutos.*;
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

    private double rotationSpeed = 1.0;

    HashMap<String, Command> eventMap;


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

    /* driver axis */
    private final int driverLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int driverRightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    
    /* driver triggers */
    final Trigger driverLeftTrigger = new Trigger(() -> driver.getRawAxis(driverLeftTriggerAxis) > 0.1);
    final Trigger driverRightTrigger = new Trigger(() -> driver.getRawAxis(driverRightTriggerAxis) > 0.1);
 
    /* Operator Buttons */
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorLB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorRStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton operatorUpStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton operatorDownStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);

    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final POVButton operatorDpadUp = new POVButton(operator, 0);
    private final POVButton operatorDpadRight = new POVButton(operator, 90);
    private final POVButton operatorDpadDown = new POVButton(operator, 180);
    private final POVButton operatorDpadLeft = new POVButton(operator, 270);

    /* operator axis */
    private final int operatorLeftYAxis = XboxController.Axis.kLeftY.value;
    private final int operatorRightYAxis = XboxController.Axis.kRightY.value;
    private final int operatorLeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;
    private final int operatorRightTriggerAxis = XboxController.Axis.kRightTrigger.value;

    /* operator triggers */
    final Trigger elevatorManualUpTrigger = new Trigger(() -> -operator.getRawAxis(operatorLeftYAxis) > 0.5);
    final Trigger elevatorManualDownTrigger = new Trigger(() -> -operator.getRawAxis(operatorLeftYAxis) < -0.5);
    final Trigger purpleLEDTrigger = new Trigger(() -> operator.getRawAxis(operatorLeftTriggerAxis) > 0.1);
    final Trigger yellowLEDTrigger = new Trigger(() -> operator.getRawAxis(operatorRightTriggerAxis) > 0.1);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    public final LEDs leds = new LEDs();
    private final Intake intake = new Intake();
    public final Wrist Wrist = new Wrist();
    private final Elevator elevator = new Elevator();
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(15);
    private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(15);

    private double targetRotation;
    
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    
    // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Wrist.setDefaultCommand(new MoveWristManual(Wrist, driver));
        leds.setDefaultCommand(new IdleLEDS(leds));
        SmartDashboard.putBoolean("isDefault", true);
        SmartDashboard.putBoolean("isPurple", false);
        SmartDashboard.putBoolean("isYellow", false);

         s_Swerve.setDefaultCommand(
             new TeleopSwerve(
                 s_Swerve, 
                 () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                 () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                 () -> -driver.getRawAxis(driverRightX), 
                 () -> driverDpadUp.getAsBoolean(),
                 () -> s_Swerve.getYaw().getDegrees(),
                 () -> driverLeftTrigger.getAsBoolean(),
                 rotationSpeed,
                 false
             )
         );

        // Configure the button bindings
        configureButtonBindings();

        eventMap = new HashMap<>();
    }

    // /**
    //  * Calculates the target rotation for the swerve drive and returns it as a DoubleSupplier
    //  */
    // private DoubleSupplier calculateTargetRotation() {
    //     return () -> {
    //         if (driverRStick.getAsBoolean()) {
    //             targetRotation = Constants.ROTATE_TO_SCORE_TARGET_ANGLE;
    //         }

    //         return targetRotation;
    //     };
    // }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        SwerveHandler();
        DriverHandler();
        OperatorHandler();

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

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return s_Swerve.followTrajectoryCommand(traj, isFirstPath);
    }

    // autos

    public Command ConePreloadSelect() {

        return new ConePreloadAuto(elevator, Wrist, intake);
    }

    public Command CubePreloadSelect() {

        return new CubePreloadAuto(elevator, Wrist, intake);
    
    }

    public Command InsideAutoConeSelect() {

        return new InsideAutoCone(elevator, Wrist, intake, s_Swerve);
    }

    public Command InsideAutoCubeSelect() {

        return new InsideAutoCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command MidAutoConeSelect() {

        return new MidAutoCone(elevator, Wrist, intake, s_Swerve);
    }

    public Command MidAutoCubeSelect() {

        return new MidAutoCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command OutsideAutoConeSelect() {

        return new OutsideAutoCone(elevator, Wrist, intake, s_Swerve);
    }

    public Command OutsideAutoCubeSelect() {

        return new OutsideAutoCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command InsideAutoBalanceConeSelect() {

        return new InsideAutoBalanceCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command InsideAutoBalanceCubeSelect() {

        return new InsideAutoBalanceCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command OutsideAutoBalanceConeSelect() {

        return new OutsideAutoBalanceCone(elevator, Wrist, intake, s_Swerve);
    }

    public Command OutsideAutoBalanceCubeSelect() {

        return new OutsideAutoBalanceCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command DuluthAutoConeSelect() {

        return new DuluthAutoCone(elevator, Wrist, intake, s_Swerve);
    }

    public Command DuluthAutoCubeSelect() {

        return new DuluthAutoCube(elevator, Wrist, intake, s_Swerve);
    }

    public Command TestAutoSelect() {

        return new TestAuto(elevator, Wrist, intake, s_Swerve);
    }
    

    public void DriverHandler() {

        driverX.whileTrue(new RunIntake(intake));
        driverA.whileTrue(new ReverseIntake(intake));

    }

    public void OperatorHandler() {

        operatorLB.onTrue(new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CONE)));
        operatorRB.onTrue(new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CUBE)));

        operatorA.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.LOW)));
        operatorB.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.MID)));
        operatorY.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.HIGH)));
        operatorBack.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.SINGLE)));
        operatorStart.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.DOUBLE)));
        operatorX.onTrue(new InstantCommand(() -> RobotMode.SetState(RobotMode.StateOptions.STOW)));
        
    }




    public void SwerveHandler() {

        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(Constants.GRYO_OFFSET)));
        driverB.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        driverStart.onTrue(new InstantCommand(() -> rotationSpeed = 0.5));
        driverBack.onTrue(new InstantCommand(() -> rotationSpeed = 1.0));

        driverDpadLeft.toggleOnTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverDpadUp.getAsBoolean(),
                () -> Constants.ROTATE_TO_SCORE_TARGET_ANGLE,
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).until(() -> Math.abs(s_Swerve.getYaw().getDegrees() % 360) < Constants.ROTATE_TO_SCORE_TARGET_ANGLE + Constants.AUTO_ROTATE_DEADBAND && 
                Math.abs(s_Swerve.getYaw().getDegrees() % 360) > Constants.ROTATE_TO_SCORE_TARGET_ANGLE - Constants.AUTO_ROTATE_DEADBAND)
        );

        driverRightTrigger.toggleOnTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverDpadUp.getAsBoolean(),
                () -> Constants.ROTATE_TO_LOAD_TARGET_ANGLE,
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).until(() -> s_Swerve.getYaw().getDegrees() % 360 < Constants.ROTATE_TO_LOAD_TARGET_ANGLE + Constants.AUTO_ROTATE_DEADBAND && 
                s_Swerve.getYaw().getDegrees() % 360 > Constants.ROTATE_TO_LOAD_TARGET_ANGLE - Constants.AUTO_ROTATE_DEADBAND)
        );
    }
   

    public void idleAnimation() {
        new InstantCommand(() -> new IdleLEDS(leds));
    }




 

    // Put buttons for testing/configuring/debugging purposes here
    public void testHandler() {

        //new GetAngle(s_Swerve);

    }
}

