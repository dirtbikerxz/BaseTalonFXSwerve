// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  private static CANSparkMax kickerMotor;
  public KickerSubsystem() {
   // kickerMotor.restoreFactoryDefaults();
    kickerMotor = new CANSparkMax(14, MotorType.kBrushless);
    kickerMotor.restoreFactoryDefaults();
    kickerMotor.setInverted(false);
    kickerMotor.setIdleMode(IdleMode.kBrake);
    kickerMotor.setSmartCurrentLimit(80);
    kickerMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setKickerSpeed(double kickerVoltage) {
    kickerMotor.setVoltage(kickerVoltage);
  }
}
