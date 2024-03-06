package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class GamerLights extends SubsystemBase {
  
  private double currentColor;

  Spark blinkin = new Spark(0); //PWM port of the driver

  public GamerLights() {}

  @Override
  public void periodic() { blinkin.set(currentColor); }

  //IF NEED COLOR DURING DRIVING
 // public void driveLight() { currentColor = -0.11; } 

  //IF NEED COLOR DURING BUTTON PRESS 
  public void buttonLight() { currentColor = -0.05; } 

  /** Idle Light; Shifts from blue (0.85) to yellow (0.67) every second */
  public void idleLight() { 
    currentColor = 0.53; 
  }
}