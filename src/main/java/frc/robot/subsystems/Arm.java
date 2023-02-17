// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

  private CANSparkMax armMotor;
  private CANCoder armEncoder;

  private double targetArmAngle;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = new CANCoder(Constants.ARM_ENCODER_ID);
  }

  /* Sets the targetArmAngle in degrees */
  public void setTargetArmAngle(double degrees) {
    targetArmAngle = degrees;
  }

  public double getArmAngle() {
    return armEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
