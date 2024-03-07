package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

//Manual: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 

public class GamerLights extends SubsystemBase {
  
  private double currentColor;

  Spark blinkin = new Spark(0); //PWM port of the driver

  public GamerLights() {}

  @Override
  public void periodic() { blinkin.set(currentColor); }

  //IF NEED COLOR DURING DRIVING; strobe red
  public void driveLight() { currentColor = -0.11; } 

  //IF NEED COLOR DURING BUTTON PRESS; strobe white
  public void buttonLight() { currentColor = -0.05; } 

  /** Idle Light; Shifts from color 1 to color 2 (set manually on the driver) */
  public void idleLight() { currentColor = 0.53; }
}