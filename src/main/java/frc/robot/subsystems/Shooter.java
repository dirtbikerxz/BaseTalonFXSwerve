package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;


public class Shooter extends SubsystemBase {
    private final TalonSRX shooterControllerL;
    private final TalonSRX shooterControllerR;

    public Shooter() {
        shooterControllerL = new TalonSRX(leftMotorID); 
        shooterControllerR = new TalonSRX(rightMotorID); 
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Speed", maxSpeed);

  // Color Sensor Code
        final I2C.Port i2cPort = I2C.Port.kOnboard;
        final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
        final ColorMatch m_colorMatcher = new ColorMatch();
  //Calibrating sensor and defining colors
        final Color kOrangeTarget = new Color(0.573, 0.354, 0.078);
        m_colorMatcher.addColorMatch(kOrangeTarget);
        Color detectedColor = m_colorSensor.getColor();
        int proximity = m_colorSensor.getProximity();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        boolean ringPresent = false; 

        if (match.color == kOrangeTarget && proximity>200){
          ringPresent = true;
        } else {
          ringPresent = false;
        }
    
        SmartDashboard.putBoolean("Ring Present", ringPresent);
    }    
    //     if (ringPresent == true){
    //       Shooter m_subsystem;
    //       init m_subsystem
    //       m_subsystem.runShooter();
    //       // schedule(ShooterCommand());
    //     }
    // }


    public void runShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, maxSpeed);
        shooterControllerR.set(ControlMode.PercentOutput, maxSpeed);
    }

    public void stopShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, 0);
        shooterControllerR.set(ControlMode.PercentOutput, 0);
    }

}