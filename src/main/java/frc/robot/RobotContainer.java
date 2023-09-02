package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private static final Swerve drivetrain = new Swerve();

    private static final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();
        configureDefaultCommands();
    }

    private void configureBindings() {
        // new Trigger(driver::getAButton).onTrue(new InstantCommand(drivetrain::resetModulesToAbsolute));
        new Trigger(driver::getXButton).whileTrue(new InstantCommand(drivetrain::park, drivetrain));

    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopSwerve(drivetrain, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> (driver.getRightTriggerAxis() > 0.75)));
    }

    public Command getAutonomousCommand() { return null;}
}
