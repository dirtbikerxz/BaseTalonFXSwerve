package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

    private double rotationSpeed = 1.0;


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
    
    /* driver triggers */
    final Trigger slowModeTrigger = new Trigger(() -> driver.getRawAxis(driverLeftTriggerAxis) > 0.1);
 
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
    public final Arm arm = new Arm();
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
        //arm.setDefaultCommand(new MoveArmManual(arm, driver));
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
                 rotationSpeed,
                 () -> false
             )
         );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Calculates the target rotation for the swerve drive and returns it as a DoubleSupplier
     */
    private DoubleSupplier calculateTargetRotation() {
        return () -> {
            if (driverRStick.getAsBoolean()) {
                targetRotation = Constants.ROTATE_TO_SCORE_TARGET_ANGLE;
            }

            return targetRotation;
        };
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

    // 25 inches
    public Command ScoreCubePreload() {

        return new SequentialCommandGroup(

            //new CloseIntake(intake).withTimeout(0.1),
            new InstantCommand(() -> s_Swerve.zeroGyro(180.0)),

            new RunIntakeAtSpeed(intake, 0.5).withTimeout(0.1),

            // move elevator to safe position
            elevator.SetElevatorPosition(Constants.ELEVATOR_SAFE_LEVEL),
            elevator.ElevatorAtPosition(),

            //move arm and elevator to scoring position
            new ParallelCommandGroup(elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL), arm.SetArmPosition(Constants.ARM_HIGH_POSITION)),    
            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // drive forward
            new DriveCommand(s_Swerve, 0.5, 0.0, 0.0).withTimeout(1.4),
            new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),

            new ConfirmScore(arm, elevator).withTimeout(0.1),
            
            // move down
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new ConfirmScore(arm, elevator)),
            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // open intake
            new ParallelCommandGroup(
                new RunIntakeAtSpeed(intake, -0.5)
            ).withTimeout(0.5),

            //move up
            new ReturnFromScoring(arm, elevator).withTimeout(0.5),

            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // drive backward
            new DriveCommand(s_Swerve, -0.5, 0.0, 0.0).withTimeout(1.4),
            new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),        

            GoToStow()
            // move arm to stow position
            // arm.SetArmPosition(Constants.ARM_STOW_POSITION), 
            // arm.ArmAtPosition(),

            // // move elevator to safe position
            // elevator.SetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL),
            // elevator.ElevatorAtPosition()
            
        );
    }

    // 27 inches
    public Command ScoreConePreload() {

        return new SequentialCommandGroup(

            new InstantCommand(() -> s_Swerve.zeroGyro(180.0)),

            new RunIntakeAtSpeed(intake, 0.5).withTimeout(0.1),

            new CloseIntake(intake).withTimeout(0.1),

            // move elevator to safe position
            elevator.SetElevatorPosition(Constants.ELEVATOR_SAFE_LEVEL),
            elevator.ElevatorAtPosition(),

            //move arm and elevator to scoring position
            new ParallelCommandGroup(elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL), arm.SetArmPosition(Constants.ARM_HIGH_POSITION)),    
            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // drive forward
            new DriveCommand(s_Swerve, 0.5, 0.0, 0.0).withTimeout(1.4),
            new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),

            new ConfirmScore(arm, elevator).withTimeout(0.1),
            
            // move down
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new ConfirmScore(arm, elevator)),
            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // open intake
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.Retract()),
                new RunIntakeAtSpeed(intake, -0.5)
            ).withTimeout(0.1),

            //move up
            new ReturnFromScoring(arm, elevator).withTimeout(0.5),

            elevator.ElevatorAtPosition(),
            arm.ArmAtPosition(),

            // drive backward
            new DriveCommand(s_Swerve, -0.5, 0.0, 0.0).withTimeout(1.4),
            new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),        

            // move arm to stow position
            arm.SetArmPosition(Constants.ARM_STOW_POSITION), 
            arm.ArmAtPosition(),

            // move elevator to safe position
            elevator.SetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL),
            elevator.ElevatorAtPosition()
            
        );
    }

    public Command ReverseBalance() {

        return new SequentialCommandGroup(

            // drive backwards
            new DriveCommand(s_Swerve, -1.0,  0.0, 0.0).withTimeout(1.25),
            new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),

            new AutoBalance(s_Swerve)

        );
    }
    

    public Command CubeAutoBalance() {

        return new SequentialCommandGroup(

        ScoreCubePreload().withTimeout(8.0),
        ReverseBalance()

    );

    }

    public Command ConeAutoBalance() {

        return new SequentialCommandGroup(

            ScoreConePreload().withTimeout(8.0),
            ReverseBalance()

        );
    }

    public Command RedLeftAutoCube() {

        PathPlannerTrajectory RedRightAutoCubePath = PathPlanner.loadPath("red left auto cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC));

        return new SequentialCommandGroup(

        ScoreCubePreload(),
        followTrajectoryCommand(RedRightAutoCubePath, true)
        //new DriveCommand(s_Swerve, 0.5, -0.5, 0.0).withTimeout(2.0),
        //new DriveCommand(s_Swerve, -1.0, 0.0, 0.0).withTimeout(3.0)

    );
    }
    
    public Command BlueRightAutoCube() {

        PathPlannerTrajectory BlueRightAutoCubePath = PathPlanner.loadPath("Blue Right Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC));

        return new SequentialCommandGroup(

        ScoreCubePreload(),
        followTrajectoryCommand(BlueRightAutoCubePath, true)
        //new DriveCommand(s_Swerve, 0.5, -0.5, 0.0).withTimeout(2.0),
        //new DriveCommand(s_Swerve, -1.0, 0.0, 0.0).withTimeout(3.0)

    );
    }

    public Command BlueLeftAutoCube() {

        PathPlannerTrajectory BlueLeftAutoCubePath = PathPlanner.loadPath("Blue Left Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC));

        return new SequentialCommandGroup(

        ScoreCubePreload(),
        followTrajectoryCommand(BlueLeftAutoCubePath, true)
        //new DriveCommand(s_Swerve, 0.5, -0.5, 0.0).withTimeout(2.0),
        //new DriveCommand(s_Swerve, -1.0, 0.0, 0.0).withTimeout(3.0)

    );
    }

    public Command RightAutoCone() {

        //PathPlannerTrajectory RightAutoConePath = PathPlanner.loadPath("Right Auto Cone", pathConstraints);

        return new SequentialCommandGroup(

        ScoreConePreload().withTimeout(8.0),

        new DriveCommand(s_Swerve, -2.0,  0.0, 0.0).withTimeout(1.5),
        new DriveCommand(s_Swerve, 0.0,  0.0, 0.0).withTimeout(0.1)

    );   
    }

    public Command LeftAutoCube() {

        return new SequentialCommandGroup(

        ScoreCubePreload()

    );
    }

    public Command LeftAutoCone() {

        return new SequentialCommandGroup(

        ScoreConePreload().withTimeout(8.0),

        //followTrajectoryCommand(examplePath, true)
        new DriveCommand(s_Swerve, -2.0,  0.0, 0.0).withTimeout(1.5),
        new DriveCommand(s_Swerve, 0.0,  0.0, 0.0).withTimeout(0.1)
    );   
    }

    public Command ScoreCone() {

        return new SequentialCommandGroup(

        ScoreConePreload()

        //followTrajectoryCommand(examplePath, true)
    );   
    }

    public Command ScoreCube() {

        return new SequentialCommandGroup(

        ScoreCubePreload()

        //followTrajectoryCommand(examplePath, true)
    );   
    }


    public Command pathTest() {

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Mid Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC));

        return new SequentialCommandGroup(
            
            ScoreCubePreload().withTimeout(8.0),
            followTrajectoryCommand(examplePath, true),
            new AutoBalance(s_Swerve)
        );
    }
    
    public void intakeHandler() {

        driverRB.whileTrue(new OpenIntake(intake));
        driverLB.whileTrue(new CloseIntake(intake));

        driverX.whileTrue(new RunIntake(intake));
        driverA.whileTrue(new ReverseIntake(intake));
        // compressor
        operatorBack.onTrue(new DisableCompressor());
    }

    public void lightHandler() {

        purpleLEDTrigger.whileTrue(new PurpleLED(leds));
        yellowLEDTrigger.whileTrue(new YellowLED(leds));

    }

    public void swerveHandler() {

        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(Constants.GRYO_OFFSET)));
        driverB.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        driverStart.onTrue(new InstantCommand(() -> rotationSpeed = 0.5));
        driverBack.onTrue(new InstantCommand(() -> rotationSpeed = 1.0));

        driverRStick.toggleOnTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverDpadUp.getAsBoolean(),
                () -> Constants.ROTATE_TO_SCORE_TARGET_ANGLE,
                rotationSpeed,
                () -> false
            )
        );
    }
   

    public void idleAnimation() {
        new InstantCommand(() -> new IdleLEDS(leds));
    }

    public void elevatorHandler() {

        //positions
        operatorY.onTrue(new ParallelCommandGroup(
            elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL), 
            arm.SetArmPosition(Constants.ARM_HIGH_POSITION)
        ));

        operatorB.onTrue(new SequentialCommandGroup(
            elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL), 
            elevator.ElevatorAtPosition(),
            new ParallelCommandGroup(
                elevator.SetElevatorPosition(Constants.ELEVATOR_MID_LEVEL), 
                arm.SetArmPosition(Constants.ARM_HIGH_POSITION)
            )
        ));

        operatorRB.onTrue(new SequentialCommandGroup(
            elevator.SetElevatorPosition(Constants.ELEVATOR_HIGH_LEVEL), 
            elevator.ElevatorAtPosition(),
            new ParallelCommandGroup(
                elevator.SetElevatorPosition(Constants.ELEVATOR_LOADING_POSITION), 
                arm.SetArmPosition(Constants.ARM_HIGH_POSITION)
            )
        ));

        
        //elevator manual
        elevatorManualUpTrigger.whileTrue(new ManualUp(elevator));
        elevatorManualDownTrigger.whileTrue(new ManualDown(elevator));


    }

    private Command GoToGround() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                elevator.SetElevatorPosition(Constants.ELEVATOR_SAFE_LEVEL),
                elevator.ElevatorAtPosition(), 
                arm.SetArmPosition(Constants.ARM_LOW_POSITION),
                arm.ArmAtPosition(),
                elevator.SetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL),
                new WaitCommand(0.5),
                arm.SetArmPosition(Constants.ARM_GROUND_POSITION)
            ),
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.SetArmPosition(Constants.ARM_LOW_POSITION),
                    elevator.SetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL)
                ),
                elevator.ElevatorAtPosition(),
                arm.SetArmPosition(Constants.ARM_GROUND_POSITION)
            ),

            () -> arm.isSafeToGround()
        );
    }

    // Might be used later
    private Command GoToStow() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevator.SetElevatorPosition(Constants.ELEVATOR_SAFE_LEVEL), 
                arm.SetArmPosition(Constants.ARM_STOW_POSITION)
            ),
            elevator.ElevatorAtPosition(), 
            arm.ArmAtPosition(), 
            elevator.SetElevatorPosition(Constants.ELEVATOR_STOW_LEVEL), 
            elevator.ElevatorAtPosition()
        );
    }

    public void armHandler() {

        //arm manual
        final Trigger armManualUpTrigger = new Trigger(() -> -operator.getRawAxis(operatorRightYAxis) > 0.5);
        final Trigger armManualDownTrigger = new Trigger(() -> -operator.getRawAxis(operatorRightYAxis) < -0.5);

        // arm.setDefaultCommand(new SetArmPosition(arm, Constants.ARM_STOW_POSITION));
        
        //Stow
        operatorStart.onTrue(GoToStow());
        // Ground
        operatorA.onTrue(GoToGround());

        //Arm Manual
        armManualUpTrigger.whileTrue(new MoveArmUp(arm));
        armManualDownTrigger.whileTrue(new MoveArmDown(arm));

        //Confirming Scores
        
        //High Score
        // operatorLB.onTrue(arm.SetArmPosition(Constants.ARM_MID_POSITION));
        // operatorLB.onFalse(arm.SetArmPosition(Constants.ARM_HIGH_POSITION));

        // //Mid Score
        // operatorRB.onTrue(elevator.SetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL));
        // operatorRB.onFalse(elevator.SetElevatorPosition(Constants.ELEVATOR_MID_LEVEL));

        // //Auto based on position
        operatorLB.onTrue(new ConfirmScore(arm, elevator));
        operatorLB.onFalse(new ReturnFromScoring(arm, elevator));
    }

    // Put buttons for testing/configuring/debugging purposes here
    public void testHandler() {

        //new GetAngle(s_Swerve);

    }
}

