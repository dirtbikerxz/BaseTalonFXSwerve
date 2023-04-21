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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IdleLEDS;
import frc.robot.commands.RunIntake;
import frc.robot.commands.MovementCommands.GoToStow;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.Timer;
//import frc.lib.util.AbsoluteEncoder;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

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

  
    /* Logging objects */
    private DataLog logger;
    private DoubleLogEntry loopTime;
    private Timer timer;
    private double previousTime;

 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Start recording all network table data
    DataLogManager.start();

    logger = DataLogManager.getLog();

    // Start recording all DS control and joystick data
    DriverStation.startDataLog(logger);

    loopTime = new DoubleLogEntry(logger, "Swerve/loopTime");
    timer = new Timer();
    previousTime = 0;

    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    SmartDashboard.putBoolean("Compressor", true);

    m_robotContainer.Wrist.SetWristPosition(Constants.WRIST_DEFAULT_STOW_POSITION);
    


    // driver camera
    final UsbCamera usbCamera = CameraServer.startAutomaticCapture();
    
    if (isReal()) {
      usbCamera.setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 160, 120, 30));
    }

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
    if (previousTime == 0) { // If it is the first loop
      timer.start();
      previousTime = timer.get();
    }

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    loopTime.append(timer.get() - previousTime);
    previousTime = timer.get();

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

    m_robotContainer.Wrist.resetRelative();
    //m_robotContainer.Wrist.SetWristPosition(Constants.WRIST_DEFAULT_STOW_POSITION);
    m_robotContainer.setSingleSubstationTargetAngle();
    // m_robotContainer.s_Swerve.zeroGyro(180);

    m_autonomousCommand = m_robotContainer.autoChooser.getSelected();
    
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
    SmartDashboard.putData("debug/command",CommandScheduler.getInstance());
  }

  @Override
  public void teleopInit() {

    m_robotContainer.Wrist.resetRelative();
    m_robotContainer.Wrist.setStow();
    m_robotContainer.setSingleSubstationTargetAngle();
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
    
    /*Reset Absolute */
    // m_robotContainer.resetAbsolute();
    
    
    //Wrist.getPositionInDegrees();

 


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putData("debug/command",CommandScheduler.getInstance());
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
