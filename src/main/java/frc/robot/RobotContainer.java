package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

import frc.robot.RobotMode.ModeOptions;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.AutoCommand;
import frc.robot.commands.AutoCommands.AutoParts.AutoBalance;
import frc.robot.commands.AutoCommands.AutoParts.ScoreConePreload;
import frc.robot.commands.AutoCommands.AutoParts.ScoreCube;
import frc.robot.commands.AutoCommands.AutoParts.ScoreCubePreload;
import frc.robot.commands.AutoCommands.GameAutos.*;
import frc.robot.commands.MovementCommands.GoToDouble;
import frc.robot.commands.MovementCommands.GoToHigh;
import frc.robot.commands.MovementCommands.GoToHybrid;
import frc.robot.commands.MovementCommands.GoToLow;
import frc.robot.commands.MovementCommands.GoToMid;
import frc.robot.commands.MovementCommands.GoToSingle;
import frc.robot.commands.MovementCommands.GoToStandingCone;
import frc.robot.commands.MovementCommands.GoToStow;
import frc.robot.RobotMode;
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
    final Trigger operatorLeftTrigger = new Trigger(() -> operator.getRawAxis(operatorLeftTriggerAxis) > 0.1);
    final Trigger operatorRightTrigger = new Trigger(() -> operator.getRawAxis(operatorRightTriggerAxis) > 0.1);
    
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final LEDs leds = new LEDs();
    private final Intake intake = new Intake();
    public final Wrist Wrist = new Wrist();
    public final Elevator elevator = new Elevator();
    public final RobotMode robotMode = new RobotMode();
    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(15);
    private final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(15);

    private double targetRotation;

    private String m_autoSelected;
    public final SendableChooser<CommandBase> autoChooser = new SendableChooser<>();
    private double singleSubstationTargetAngle;
    
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    
    // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
    // Or the path can be sampled at a given point in time for custom path following


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //Wrist.setDefaultCommand(new MoveWristManual(Wrist, driver));
        SmartDashboard.putBoolean("isDefault", true);
        SmartDashboard.putBoolean("isPurple", false);
        SmartDashboard.putBoolean("isYellow", false);

         s_Swerve.setDefaultCommand(
             new TeleopSwerve(
                 s_Swerve, 
                 () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                 () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                 () -> -driver.getRawAxis(driverRightX), 
                 () -> driverRightTrigger.getAsBoolean(),
                 () -> s_Swerve.getYaw().getDegrees(),
                 () -> driverLeftTrigger.getAsBoolean(),
                 rotationSpeed,
                 false
             )
         );

        eventMap = new HashMap<>();



        eventMap.put("Score Cone Preload", new ScoreConePreload(elevator, Wrist, intake));
        eventMap.put("Score Cube Preload", new ScoreCubePreload(elevator, Wrist, intake));
        eventMap.put("Score Cube", new ScoreCube(elevator, Wrist, intake));
        eventMap.put("Auto Balance", new AutoBalance(s_Swerve));
        eventMap.put("Go To Ground", new GoToLow(Wrist, elevator));
        eventMap.put("Run Intake", new RunIntake(intake));
        eventMap.put("Reverse Intake", new ReverseIntake(intake));
        eventMap.put("Go To Stow", new GoToStow(Wrist, elevator));




        //autoChooser.addOption("Mid Auto Timed", new AutoCommand(DuluthAuto2()));
        //autoChooser.addOption("Score Preload Timed", new AutoCommand(new ScoreCubePreload(elevator, Wrist, intake))); 

        //autoChooser.addOption("Score Cone Preload",new AutoCommand(s_Swerve, "Score Cone Preload", eventMap));

        //autoChooser.addOption("Inside Auto Cone",new AutoCommand(s_Swerve, "Inside Auto Cone", eventMap));
        autoChooser.addOption("Inside Auto Cube",new AutoCommand(s_Swerve, "Inside Auto Cube", eventMap));

        //autoChooser.addOption("Mid Auto Cone",new AutoCommand(s_Swerve, "Mid Auto Cone", eventMap));
        //autoChooser.addOption("Mid Auto Cube",new AutoCommand(s_Swerve, "Mid Auto Cube", eventMap));
        autoChooser.setDefaultOption("Mid Auto Cube",new AutoCommand(s_Swerve, "Mid Auto Cube 2", eventMap)); //NOTE: mid auto 2

        //autoChooser.addOption("Outside Auto Cone",new AutoCommand(s_Swerve, "Outside Auto Cone", eventMap));
        autoChooser.addOption("Outside Auto Cube",new AutoCommand(s_Swerve, "Outside Auto Cube", eventMap));

        //autoChooser.addOption("Inside Auto Balance Cone",new AutoCommand(s_Swerve, "Inside Auto Balance Cone", eventMap));
        //autoChooser.addOption("Inside Auto Balance Cube",new AutoCommand(s_Swerve, "Inside Auto Balance Cube", eventMap));

        //autoChooser.addOption("Outside Auto Balance Cone",new AutoCommand(s_Swerve, "Outside Auto Balance Cone", eventMap));
        //autoChooser.addOption("Outside Auto Balance Cube",new AutoCommand(s_Swerve, "Outside Auto Balance Cube", eventMap));

        //autoChooser.addOption("Duluth Auto Cone",new AutoCommand(s_Swerve, "Duluth Auto Cone", eventMap));
        autoChooser.addOption("Score Cube Preload",new AutoCommand(s_Swerve, "Score Cube Preload", eventMap));
        autoChooser.addOption("Duluth Auto Path",new AutoCommand(s_Swerve, "Duluth Auto Cube", eventMap));
        autoChooser.addOption("Duluth Auto Timed", new AutoCommand(DuluthAuto()));
        autoChooser.addOption("Leave Community Auto Timed", new AutoCommand(LeaveCommunityAuto()));


        // autoChooser.addOption("Score Cube Preload", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Score Cube Preload", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        
        // autoChooser.addOption("Inside Auto Cone", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Inside Auto Cone", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        // autoChooser.addOption("Inside Auto Cube", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Inside Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        
        // autoChooser.addOption("Mid Auto Balance Cone", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Mid Auto Cone", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        // autoChooser.addOption("Mid Auto Balance Cube", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Mid Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        
        // autoChooser.addOption("Outside Auto Cone", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Outside Auto Cone", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        // autoChooser.addOption("Outside Auto Cube", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Outside Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        
        // autoChooser.addOption("Inside Auto Balance Cone", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Inside Auto Balance Cone", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        // autoChooser.addOption("Inside Auto Balance Cube", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Inside Auto Balance Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        
        // autoChooser.addOption("Duluth Auto Cone", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Duluth Auto Cone", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));
        // autoChooser.addOption("Duluth Auto Cube", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Duluth Auto Cube", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));

        // autoChooser.addOption("Test Auto", autoBuilder.fullAuto(new ArrayList<PathPlannerTrajectory>(PathPlanner.loadPathGroup("Test Auto", new PathConstraints(Constants.AUTO_VEL, Constants.AUTO_ACC)))));


        SmartDashboard.putData("driver/auto", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
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

    public CommandBase DuluthAuto() {

        return new SequentialCommandGroup(

        // score preload
        new ScoreCubePreload(elevator, Wrist, intake),

        // drive backwards
        new DriveCommand(s_Swerve, -1.0,  0.0, 0.0).withTimeout(1.25),
        //stop
        new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),

        // auto-balance
        new AutoBalance(s_Swerve)

    );
    }

    public CommandBase LeaveCommunityAuto() {

        return new SequentialCommandGroup(

        // score preload
        new ScoreCubePreload(elevator, Wrist, intake),

        // drive backwards
        new DriveCommand(s_Swerve, -1.0,  0.0, 0.0).withTimeout(4)

    );
    }

    public CommandBase DuluthAuto2() {

        return new SequentialCommandGroup(

        // score preload
        new ScoreCubePreload(elevator, Wrist, intake),

        // drive backwards
        new DriveCommand(s_Swerve, -1.0,  0.0, 0.0).withTimeout(4),
        //stop
        new DriveCommand(s_Swerve, 0.0,0.0,0.0).withTimeout(0.1),
        //drive forwards
        new DriveCommand(s_Swerve, 1.0,0.0,0.0).withTimeout(2),
        // auto-balance
        new AutoBalance(s_Swerve)

    );
    }

    public void setSingleSubstationTargetAngle() {
        if (DriverStation.getAlliance() == Alliance.Red) {
            singleSubstationTargetAngle = Constants.ROTATE_TO_SINGLE_SUBSTATION_RED_TARGET_ANGLE;
        } else {
            singleSubstationTargetAngle = Constants.ROTATE_TO_SINGLE_SUBSTATION_BLUE_TARGET_ANGLE;
        }
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return s_Swerve.followTrajectoryCommand(traj, isFirstPath);
    }

    

    public void DriverHandler() {

        driverX.whileTrue(new ParallelCommandGroup(
            new RunIntake(intake), 
            new WaitCommand(0.5)
            .andThen(intake.waitUntilCurrentPast(17.0, 0.1))
                .andThen(new GreenLED(leds))
                .andThen(new WaitCommand(0.5))
                .andThen(new ConditionalCommand(
                    new YellowLED(leds), 
                    new PurpleLED(leds), 
                    () -> {
                        if (RobotMode.mode == ModeOptions.CONE) {
                            return true;
                        }
                        else {
                            return false;
                        }
                }))
            )
        ).onFalse(new ConditionalCommand(
            new YellowLED(leds), 
            new PurpleLED(leds), 
            () -> {
                if (RobotMode.mode == ModeOptions.CONE) {
                    return true;
                }
                else {
                    return false;
                }
        }));
        driverA.whileTrue(new ReverseIntake(intake));
        //driverDpadDown.onTrue(new AutoCommand(s_Swerve, "Mid Auto Cube", eventMap).withTimeout(15.0));

    }

    public void OperatorHandler() {

        // operatorLB.onTrue(new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CONE)));
        // operatorRB.onTrue(new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CUBE)));

        operatorRB.onTrue(new ConditionalCommand(
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CONE)),
                new YellowLED(leds)
            ),
            new ParallelCommandGroup(
                new InstantCommand(() -> RobotMode.SetMode(RobotMode.ModeOptions.CUBE)),
                new PurpleLED(leds)
            ),
            
            () -> {
                if (RobotMode.mode == ModeOptions.CUBE) {
                    return true;
                }
                else {
                    return false;
                }
            }
            
            )
        );

        operatorA.onTrue(new GoToLow(Wrist, elevator));
        operatorX.onTrue(new GoToMid(Wrist, elevator));
        operatorY.onTrue(new GoToHigh(Wrist, elevator));
        operatorB.onTrue(new GoToStow(Wrist, elevator));

        // operatorBack.onTrue(new GoToSingle(Wrist, elevator));
        // operatorStart.onTrue(new GoToDouble(Wrist, elevator));
        
        operatorLeftTrigger.onTrue(new GoToSingle(Wrist, elevator)).onFalse(new GoToStow(Wrist, elevator));
        operatorRightTrigger.onTrue(new GoToStandingCone(Wrist, elevator));

        operatorLB.onTrue(new GoToDouble(Wrist, elevator)).onFalse(new GoToStow(Wrist, elevator));

        //operatorDpadLeft.onTrue(new GoToStandingCone(Wrist, elevator));
        //operatorDpadRight.onTrue(new GoToHybrid(Wrist, elevator));

        elevatorManualUpTrigger.whileTrue(new ManualUp(elevator));
        elevatorManualDownTrigger.whileTrue(new ManualDown(elevator));

        
    }




    public void SwerveHandler() {


        driverY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driverB.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        driverStart.onTrue(new InstantCommand(() -> rotationSpeed = 0.5));
        driverBack.onTrue(new InstantCommand(() -> rotationSpeed = 1.0));

        driverRStick.toggleOnTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverRightTrigger.getAsBoolean(),
                () -> Constants.ROTATE_TO_SCORE_TARGET_ANGLE,
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).until(() -> Math.abs(s_Swerve.getYaw().getDegrees() % 360) < Constants.ROTATE_TO_SCORE_TARGET_ANGLE + Constants.AUTO_ROTATE_DEADBAND && 
                Math.abs(s_Swerve.getYaw().getDegrees() % 360) > Constants.ROTATE_TO_SCORE_TARGET_ANGLE - Constants.AUTO_ROTATE_DEADBAND)
        );

        driverRB.toggleOnTrue(

            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverRightTrigger.getAsBoolean(),
                () -> Constants.ROTATE_TO_DOUBLE_SUBSTATION_TARGET_ANGLE,
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).until(() -> Math.abs(s_Swerve.getYaw().getDegrees() % 360) < Constants.ROTATE_TO_DOUBLE_SUBSTATION_TARGET_ANGLE + Constants.AUTO_ROTATE_DEADBAND && 
                Math.abs(s_Swerve.getYaw().getDegrees() % 360) > Constants.ROTATE_TO_DOUBLE_SUBSTATION_TARGET_ANGLE - Constants.AUTO_ROTATE_DEADBAND)
        );

        driverLB.toggleOnTrue(

            new TeleopSwerve(
                s_Swerve, 
                () -> -slewRateLimiterY.calculate(driver.getRawAxis(driverLeftY)), 
                () -> -slewRateLimiterX.calculate(driver.getRawAxis(driverLeftX)), 
                () -> -driver.getRawAxis(driverRightX), 
                () -> driverRightTrigger.getAsBoolean(),
                () -> singleSubstationTargetAngle,
                () -> driverLeftTrigger.getAsBoolean(),
                rotationSpeed,
                true
            ).until(() -> Math.abs(s_Swerve.getYaw().getDegrees() % 360) < ((singleSubstationTargetAngle + 180) % 360) + Constants.AUTO_ROTATE_DEADBAND && 
                Math.abs(s_Swerve.getYaw().getDegrees() % 360) > ((singleSubstationTargetAngle + 180) % 360) - Constants.AUTO_ROTATE_DEADBAND)
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

