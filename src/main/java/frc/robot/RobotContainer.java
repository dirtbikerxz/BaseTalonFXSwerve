// 1/23/24 Maddie, Sam, Kaden 
//Changed Joystick to XboxController

// 1/24/24 Kaden
//added shoot subsystem and constant shooting using voltage

package frc.robot;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final XboxController driver = new XboxController(0);

    private PhotonCamera camera = new PhotonCamera("2531Limelight");

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); //map to button 7 for two squares
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton getIntoDistance = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shootVolts = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final Shoot shoot = new Shoot();

    private Supplier<Pose2d> poseSupplier;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // getIntoDistance.onTrue(new GetIntoRange(vision, s_Swerve, poseSupplier, camera));
        getIntoDistance.onTrue(new InstantCommand(() -> shoot.brakeMotors()));
        getIntoDistance.onFalse(new InstantCommand(() -> shoot.coastMotors()));


        // shootVolts.onTrue(new InstantCommand(() -> shoot.setMotorVolts(-8)));
        shootVolts.onTrue(new InstantCommand(() -> shoot.setSplitMotorVolts(-10,-10)));
        shootVolts.onFalse(new InstantCommand(() -> shoot.setMotorVolts(0)));
    
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
}
