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
    private final GamerLights s_Blinkin = new GamerLights();


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
        // s_Intake.setDefaultCommand(new IntakeCommand(s_Intake));

        //s_Blinkin.setDefaultCommand(new InstantCommand(() -> s_Blinkin.idleLight()));
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
        controls.hangExtend.whileTrue(new HangCommandUp(s_Hang));
        controls.hangRetract.whileTrue(new HangCommandDown(s_Hang));
        controls.hangNoLimits.onTrue(new InstantCommand(() -> s_Hang.removeHangLimits()));
        
        controls.requestAmplification.whileTrue(new InstantCommand(() -> s_Blinkin.idleLight()));
        controls.activateShooter.onTrue(new InstantCommand(() -> s_Shooter.runShooter()));
        controls.runIntake.whileTrue(new InstantCommand(() -> s_Intake.runIntake()));
        controls.reverseIntake.whileTrue(new InstantCommand(() -> s_Intake.reverseIntake()));
        controls.toggleIntake.whileTrue(new InstantCommand(() -> s_Intake.toggleIntake()));
        controls.slowMode.whileTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> controls.getForward() / 2, 
                () -> controls.getStrafe() / 2, 
                () -> controls.getRotation() / 2, 
                () -> controls.robotCentric.getAsBoolean()
            )
        );
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
