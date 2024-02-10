// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.doubleNeoConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public CANSparkFlex intakeMotor;

  public Intake() {
    intakeMotor = new CANSparkFlex(doubleNeoConstants.intakeMotorCAN, MotorType.kBrushless);
  }

  public void doIntake() {
    intakeMotor.set(0.9);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intakemotor", intakeMotor.get());
  }
}
