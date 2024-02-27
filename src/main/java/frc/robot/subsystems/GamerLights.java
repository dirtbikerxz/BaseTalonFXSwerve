package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Spark;

public class GamerLights extends SubsystemBase {
  
  private double currentColor;

  Spark blinkin = new Spark(0); //PWM port of the driver

  Joystick driverController = new Joystick(0);

  public GamerLights() {}

  @Override
  public void periodic() { blinkin.set(currentColor); }

  //IF NEED COLOR DURING DRIVING
  public void driveLight() { currentColor = 0.69; } 

  //IF NEED COLOR DURING BUTTON PRESS 
  public void buttonLight() { currentColor = 0.65; } 

  /** Idle Light; Shifts from blue (0.85) to yellow (0.67) every second */
  public void idleLight() { 
    currentColor = ((int) Timer.GetMatchTime()) % 2 == 0 ? 0.85 : 0.67; 
  }
}
    