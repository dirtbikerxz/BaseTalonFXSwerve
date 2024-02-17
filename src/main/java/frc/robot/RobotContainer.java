package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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
    private final DriveControls controls = new DriveControls();

    // /* Drive Controls */
    // private final int translationAxis = Joystick.Axis.kLeftY.value;
    // private final int strafeAxis = Joystick.Axis.kLeftX.value;
    // private final int rotationAxis = Joystick.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Hang s_Hang = new Hang();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> controls.getForward(), 
                () -> controls.getStrafe(), 
                () -> controls.getRotation(), 
                () -> controls.robotCentric.getAsBoolean()
            )
        );

        //this just constantly runs the intake and shooter for now:
        s_Shooter.setDefaultCommand(new ShooterCommand(s_Shooter));
        s_Intake.setDefaultCommand(new IntakeCommand(s_Intake));
        s_Hang.setDefaultCommand(new IntakeCommand(s_Hang));
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons; the names should be self-explanitory */
        controls.zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //These buttons should proabably be recoded later??
        controls.hangExtend.whileTrue(new InstantCommand(() -> s_Hang.runHang(1)));
        controls.hangRetract.whileTrue(new InstantCommand(() -> s_Hang.runHang(-1)));

        controls.activateShooter.onTrue(new InstantCommand(() -> s_Shooter.runShooter()));
        controls.runIntake.onTrue(new InstantCommand(() -> s_Intake.runIntake()));
        controls.reverseIntake.onTrue(new InstantCommand(() -> s_Intake.reverseIntake()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getShooterCommand() {
        return new ShooterCommand(s_Shooter);
    }

    public Command getIntakeCommand() {
        return new IntakeCommand(s_Intake);
    }

    public Command getAutoCommand() {
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("NewPath");

        return AutoBuilder.followPath(path);
    }
}
