// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsytem. */
  private static CANSparkMax rightShooterMotor;
  private static CANSparkMax leftShooterMotor;
  //private static CANSparkMax kickerMotor;
  public ShooterSubsystem() {
    rightShooterMotor = new CANSparkMax(20, MotorType.kBrushless);
    rightShooterMotor.restoreFactoryDefaults();
    rightShooterMotor.setInverted(true);
    rightShooterMotor.setIdleMode(IdleMode.kCoast);
    rightShooterMotor.setSmartCurrentLimit(80);
    rightShooterMotor.burnFlash();


    leftShooterMotor = new CANSparkMax(18, MotorType.kBrushless);
    leftShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.setInverted(false);
    leftShooterMotor.setIdleMode(IdleMode.kCoast);
    leftShooterMotor.setSmartCurrentLimit(80);
    leftShooterMotor.burnFlash();

   /*  kickerMotor.restoreFactoryDefaults();
    kickerMotor = new CANSparkMax(16, MotorType.kBrushless);
    kickerMotor.setInverted(false);
    kickerMotor.setIdleMode(IdleMode.kBrake);
    kickerMotor.setSmartCurrentLimit(80);
    kickerMotor.burnFlash();*/


  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
  public void setShooterSpeed(double leftVoltage, double rightVoltage) {
    leftShooterMotor.setVoltage(leftVoltage);
    rightShooterMotor.setVoltage(rightVoltage);
  }
  /*public double getRightMotorVoltage(){
    return rightShooterMotor.getBusVoltage();
  }*/

  public double getRightMotorVoltage() {
    return rightShooterMotor.getVoltageCompensationNominalVoltage();
  }

  /*public void setKickerSpeed(double kickerVoltage) {
    kickerMotor.setVoltage(kickerVoltage);
  }*/

}
