package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Swerve.ShooterConstants;
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
   // private final Joystick driver = new Joystick(0);
    
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
    private final XboxController driverXbox = new XboxController(0);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driverXbox, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
    

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverXbox.getRawAxis(translationAxis), 
                () -> -driverXbox.getRawAxis(strafeAxis), 
                () -> -driverXbox.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        shooterSubsystem.setDefaultCommand(new ShooterCommand(shooterSubsystem, 0,0));

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

        //new Trigger(driverXbox.getLeftTriggerAxis() > 0.5) .whileTrue(new ShooterCommand(shooterSubsytem, 10.0, 6.0 ));

       // driverXbox.a().whileTrue(new ShooterCommand(shooterSubsytem , 10.0,6.0));
        //new JoystickButton(driverXbox, 1).whileTrue(new ShooterCommand(shooterSubsytem, 10.0, 6.0));

       
       new JoystickButton(driverXbox, 1).whileTrue(new ShooterCommand(shooterSubsystem, 6.0, 10.0 ));
       new JoystickButton(driverXbox, 2).whileTrue(new KickerCommand(kickerSubsystem, 11.0));
       /*if(new JoystickButton(driverXbox, 1).whileTrue(new ShooterCommand(shooterSubsystem, 6.0, 10.0)) != null) {
        if(shooterSubsystem.getRightMotorVoltage() <= 10.0){
            new KickerCommand(kickerSubsystem, 11.0);

        }
       }*/

    }


    /*public void ampShooter() {
        if(driverXbox.getAButtonPressed()) {
            new ShooterCommand(shooterSubsystem, 6.0, 10.0);
                if(shooterSubsystem.getRightMotorVoltage() >= 10.0  ){
                    new KickerCommand(kickerSubsystem, 5.0);
                }
           
            

        }
    }*/
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
