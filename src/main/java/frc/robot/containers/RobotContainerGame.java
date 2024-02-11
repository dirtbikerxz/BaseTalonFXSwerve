package frc.robot.containers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Robot;
import frc.robot.commands.Arm.IntakePosition;
import frc.robot.commands.Arm.ShootPosition;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.IndexNote;
import frc.robot.commands.Shooter.ShootNote;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerGame implements RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arm a_Arm = new Arm();
    private final Intake i_Intake = new Intake();
    private final Shooter s_Shooter = new Shooter();

    // commands

    private final IntakePosition armToIntake = new IntakePosition(a_Arm);
    private final ShootPosition armToShoot = new ShootPosition(a_Arm);

    private final IntakeNote intakeNote = new IntakeNote(i_Intake);

    private final IndexNote indexNote = new IndexNote(s_Shooter);
    private final ShootNote shootNote = new ShootNote(s_Shooter);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerGame() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis),
                    robotCentric
            )
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
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        intake.onTrue(new SequentialCommandGroup(armToIntake, new ParallelDeadlineGroup(indexNote, intakeNote)));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return Commands.print("No Auto");
    }

    @Override
    public void robotPeriodic() {
        SwerveModuleState[] swerveStates = s_Swerve.getModuleStates();
        for (int i = 0; i < swerveStates.length; i++) {
            SwerveModuleState state = swerveStates[i];
            SmartDashboard.putNumber(String.format("SwerveSpeed%d", i), state.speedMetersPerSecond);
        }
    }
}
