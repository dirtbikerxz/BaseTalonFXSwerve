// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {

    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private DoubleSolenoid solenoid;
    PneumaticHub pneumaticHub;


    public Intake() {

      motor1 = new CANSparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
      motor2 = new CANSparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);
      motor1.setIdleMode(IdleMode.kBrake);
      motor1.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
      motor2.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor2.enableSoftLimit(SoftLimitDirection.kReverse, false);

      /* Current Limits */
      motor1.setSmartCurrentLimit(10);
      motor2.setSmartCurrentLimit(10);

      pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
      solenoid = pneumaticHub.makeDoubleSolenoid(Constants.PNEUMATIC_FORWARD_CHANNEL, Constants.PNEUMATIC_REVERSE_CHANEL);

    }

    public void Run(double speed) {

      motor1.set(speed);
      motor2.set(-speed);

    }

    public void Stop() {

      motor1.set(0.15);
      motor2.set(0.15);

    }

    public void Extend() {

      solenoid.set(DoubleSolenoid.Value.kForward);

    }
    
    public void Retract() {

      solenoid.set(DoubleSolenoid.Value.kReverse);
      
    }


    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

}
