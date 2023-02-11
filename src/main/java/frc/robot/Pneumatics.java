/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Pneumatics extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final Compressor _Compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final DoubleSolenoid _Charge = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 0, 1);
  private final DoubleSolenoid _Fire = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, 2, 3);
  private final XboxController _controller = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /* Rev Pneumatic Hub Double Solenoid Channels:
    (0) = High Charge
    (1) = Low Charge
    (2) = Release
    (3) = Extend/Clamp


  
  /**
   * This function is called periodically during autonomous.
   */


  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    _Fire.set(DoubleSolenoid.Value.kForward);

  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if (_controller.getAButtonPressed()){
      _Charge.set(DoubleSolenoid.Value.kForward);
      // _Fire.set(DoubleSolenoid.Value.kReverse);
      _Fire.toggle();
    }
    if (_controller.getBButtonPressed()){
      _Charge.set(DoubleSolenoid.Value.kReverse);
      // _Fire.set(DoubleSolenoid.Value.kReverse);
      _Fire.toggle();
    }
    if (_controller.getRightBumperPressed()){
      _Charge.set(DoubleSolenoid.Value.kOff);
      // _Fire.set(DoubleSolenoid.Value.kOff);
    }

  // if (_controller.getXButtonPressed()){
  //   _Fire.set(DoubleSolenoid.Value.kForward);
  // }

  // // if (_controller.getYButtonPressed()){
  // //   _Fire.set(DoubleSolenoid.Value.kReverse);
  // // }
  // if (_controller.getLeftBumperPressed()){
  //   _Fire.set(DoubleSolenoid.Value.kReverse);
  // }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}



    