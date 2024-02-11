package frc.robot.containers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Robot;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerDebug implements RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final JoystickButton commandDrive = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton commandSteer = new JoystickButton(driver, XboxController.Button.kB.value);
//    private final JoystickButton commandShoot = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final SwerveTest motorTest = new SwerveTest();
//    private final Shooter mShooter = new Shooter(3, 4);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerDebug() {
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

        //Command testCommand = makeSteeringTestCommand();
        //commandDrive.onTrue(testCommand);
//        commandSteer.onTrue(new SwerveAssignSteer(motorTest));
//        commandShoot.onTrue(new ShooterAssignPower(mShooter, 0.70));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new InstantCommand(() -> {});
    }

//    public Command makeDriveTestCommand() {
//        return new SequentialCommandGroup(
//                new SwerveAssignSpeed(motorTest, 0, 0.5).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 1, 0.5).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 2, 0.5).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 3, 0.5).withTimeout(2.0),
//
//                new SwerveAssignSpeed(motorTest, 0, 0).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 1, 0).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 2, 0).withTimeout(2.0),
//                new SwerveAssignSpeed(motorTest, 3, 0).withTimeout(2.0)
//        );
//    }
//
//    public Command makeSteeringTestCommand() {
//        return new SequentialCommandGroup(
//                new SwerveAssignSteer(motorTest, 0, 0.5).withTimeout(2.0),
//                new SwerveAssignSteer(motorTest, 1, 0.5).withTimeout(2.0),
//                new SwerveAssignSteer(motorTest, 2, 0.5).withTimeout(2.0),
//                new SwerveAssignSteer(motorTest, 3, 0.5).withTimeout(2.0)
//        );
//    }
}
