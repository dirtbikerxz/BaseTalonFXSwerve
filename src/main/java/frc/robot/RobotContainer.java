// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final Joystick driver = new Joystick(0);

  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  private Command autoCommand;

  private static final String exampleAuto = "Example Auto";
  private static final String ultrasonicAuto = "Ultrasonic Auto";

  /* Drive Controls */
  // private final int translationAxis = XboxController.Axis.kLeftY.value;
  // private final int strafeAxis = XboxController.Axis.kLeftX.value;
  // private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 2;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  private Ultrasonic ultrasonic = new Ultrasonic();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = Constants.Swerve.isFieldRelative;
    boolean openLoop = Constants.Swerve.isOpenLoop;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, ultrasonic, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    autoChooser.setDefaultOption("Example Auto", exampleAuto);
    autoChooser.addOption("Ultrasonic Auto", ultrasonicAuto);
    SmartDashboard.putData("Choose Auto: ", autoChooser);
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
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   /*
  switch (autoChooser.getSelected()) {
    case "Example Auto":
      return new exampleAuto(s_Swerve);
    case "Ultrasonic Auto":
      return new ultrasonicAuto(s_Swerve);
  }
  */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // autoCommand = new exampleAuto(s_Swerve);
    // switch (autoChooser.getSelected()) {
    //   case "Example Auto":
    //     System.out.println("Example Auto!!!!!!!!!!!!!!");
    //   case "Ultrasonic Auto":
    //     System.out.println("Ultrasonic Auto!!!!!!!!!!!!!!");
    // }

    if(autoChooser.getSelected() == "Example Auto"){
      System.out.println("Example Auto!!!!!!!!!!!!!!");
      autoCommand = new exampleAuto(s_Swerve);
    } else if (autoChooser.getSelected() == "Ultrasonic Auto"){
      System.out.println("Ultrasonic Auto!!!!!!!!!!!!!!");
      autoCommand = new ultrasonicAuto(s_Swerve, ultrasonic);
    }
    // // return new exampleAuto(s_Swerve);
    return autoCommand;
    
  }
}
