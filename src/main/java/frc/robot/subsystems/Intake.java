// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public CANSparkMax intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkMax(Constants.IntakeMototCANID, MotorType.kBrushless);
  }

  public void IntakeFast1() {
    intakeMotor.set(0.9);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
