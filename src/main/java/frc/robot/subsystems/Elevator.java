// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
    private CANSparkMax elevatorMotor;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.FORWARD_ELEVATOR_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.REVERSE_ELEVATOR_LIMIT);
    
    }

    public void extend() {
        elevatorMotor.set(0.25);
    }

    public void retract() {
        elevatorMotor.set(-0.25);
    }

    public void stop() {
        elevatorMotor.set(0);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
