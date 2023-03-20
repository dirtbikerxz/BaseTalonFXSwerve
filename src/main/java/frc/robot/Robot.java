// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IdleLEDS;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.Timer;
//import frc.lib.util.AbsoluteEncoder;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Command resetAbsolute;

  private static final String auto1 = "Left Auto Cube";
  private static final String auto2 = "Left Auto Cone";
  private static final String auto3 = "Mid Auto Cube";
  private static final String auto4 = "Mid Auto Cone";
  private static final String auto5 = "Red Left Auto Cube";
  private static final String auto6 = "Blue Right Auto Cube";
  private static final String auto7 = "Right Auto Cone";
  private static final String auto8 = "Cube Preload";
  private static final String auto9 = "Cone Preload";
  private static final String auto10 = "Test Auto";
  private static final String auto11 = "Blue Left Auto Cube";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putBoolean("Compressor", true);

    m_robotContainer.arm.SetArmPosition(Constants.ARM_STOW_POSITION);
    
    m_chooser.addOption("Left Auto Cube", auto1);
    m_chooser.addOption("Left Auto Cone", auto2);

    m_chooser.addOption("Mid Auto Cube", auto3);
    m_chooser.setDefaultOption("Mid Auto Cone", auto4);

    m_chooser.addOption("Red Left Auto Cube", auto5);
    m_chooser.addOption("Blue Right Auto Cube", auto6);
    m_chooser.addOption("Right Auto Cone", auto7);

    m_chooser.addOption("Cube Preload", auto8);
    m_chooser.addOption("Cone Preload", auto9);

    m_chooser.addOption("Test Auto", auto10);
    m_chooser.addOption("Blue Left Auto Cube", auto11);

    SmartDashboard.putData("Auto Choices", m_chooser);

    // driver camera
    final UsbCamera usbCamera = CameraServer.startAutomaticCapture();
    
    usbCamera.setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 160, 120, 30));

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    /*Reset Absolute */
    m_robotContainer.resetAbsolute();
    m_robotContainer.arm.resetRelative();
    //m_robotContainer.MidAuto();

    m_robotContainer.arm.SetArmPosition(Constants.ARM_STOW_POSITION);

    m_autoSelected = m_chooser.getSelected();

    switch (m_autoSelected) {

      case auto1:
        m_autonomousCommand = m_robotContainer.LeftAutoCube();
        break;
      case auto2:
        m_autonomousCommand = m_robotContainer.LeftAutoCone();
        break;
      case auto3:
        m_autonomousCommand = m_robotContainer.CubeAutoBalance();
        break;
      case auto4:
        m_autonomousCommand = m_robotContainer.ConeAutoBalance();
        break;
      case auto5:
        m_autonomousCommand = m_robotContainer.RedLeftAutoCube();
        break;
      case auto6:
        m_autonomousCommand = m_robotContainer. BlueRightAutoCube();
        break;
      case auto7:
        m_autonomousCommand = m_robotContainer.RightAutoCone();
        break;
      case auto8:
        m_autonomousCommand = m_robotContainer.ScoreCubePreload();
      case auto9:
        m_autonomousCommand = m_robotContainer.ScoreConePreload();
      case auto10:
        m_autonomousCommand = m_robotContainer.pathTest();
      case auto11:
        m_autonomousCommand = m_robotContainer.BlueLeftAutoCube();
      default:
        m_autonomousCommand = m_robotContainer.ScoreCubePreload();

    }

    
    // PathConstraints pathConstraints = new PathConstraints(4, 3);
    
    // // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("New New New New Path", pathConstraints);
    
    // // This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
    // // Or the path can be sampled at a given point in time for custom path following
      
    // // Sample the state of the path at 1.2 seconds
    // PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);
    
    // // Print the velocity at the sampled time
    // System.out.println(exampleState.velocityMetersPerSecond);
    // System.out.println(exampleState.velocityMetersPerSecond);
    // System.out.println(exampleState.velocityMetersPerSecond);
    
    // m_autonomousCommand = m_robotContainer.followTrajectoryCommand(examplePath, true);

    //m_autonomousCommand = m_robotContainer.pathTest();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  }

  @Override
  public void teleopInit() {

    m_robotContainer.arm.resetRelative();
    m_robotContainer.arm.SetArmPosition(Constants.ARM_STOW_POSITION);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
    
    /*Reset Absolute */
    m_robotContainer.resetAbsolute();
    
    
    //arm.getPositionInDegrees();

 


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
