// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  
    private RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorMotor;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);

        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.FORWARD_ELEVATOR_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.REVERSE_ELEVATOR_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder(); 
        elevatorMotor.setIdleMode(IdleMode.kBrake);
    }

    public void extend() {
        elevatorMotor.set(0.25);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    }

    public void retract() {
        elevatorMotor.set(-0.25);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    }

    public void stop() {
        elevatorMotor.set(0);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    } 
    
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    }
}
