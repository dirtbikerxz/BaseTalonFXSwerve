// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {
  
  private TalonFX motor1 = new TalonFX(31);
  private TalonFX motor2 = new TalonFX(22);

  public Shoot() {
    motor1.setInverted(false);
    motor2.setInverted(true);
    // pivotMotor.setInverted(false);

    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
    // pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    motor1.getConfigurator().apply(new TalonFXConfiguration());
    motor2.getConfigurator().apply(new TalonFXConfiguration());
    // pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    // PIDController shootingPIDController = new PIDController(0.2, 0.002, 0);

    var slot0Configs = new Slot0Configs();
      slot0Configs.kV = 0.12;
      slot0Configs.kP = 0.11;
      slot0Configs.kI = 0.00048;
      slot0Configs.kD = 0.0;
      motor1.getConfigurator().apply(slot0Configs, 0.050);
      motor2.getConfigurator().apply(slot0Configs, 0.050);
  }


  public Command RunMotors(double speed) {
    return run(
        () -> {
         setMotorSpeed(speed);
        });
  }

  public Command RunMotorVoltage(double rpm) {
    return run(() -> {
      setMotorVolts(RPMToVolts(rpm));
      System.out.println("Motor1 RPM = " + getRPMfromVelocity1() + "Motor2 RPM" + getRPMfromVelocity2());
    });
  }

  public void setMotorSpeed(double speed) {
    motor1.set(speed);
    motor2.set(speed);
  }

  public void setSplitMotorSpeed(double one, double two) {
    motor1.set(one);
    motor2.set(two);
  }

  public void setSplitMotorVolts(double one, double two) {
    motor1.setVoltage(one);
    motor2.setVoltage(two);
  }

  public void setMotorVolts(double voltage) {
    motor1.setVoltage(voltage);
    motor2.setVoltage(voltage);
  }

  public void brakeMotors() {
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }

  public void coastMotors() {
    motor1.setNeutralMode(NeutralModeValue.Coast);
    motor2.setNeutralMode(NeutralModeValue.Coast);
  }

  public void stopMotors() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  public double getRPMfromVelocity1() {
    // return (motor1.getVelocity().getValueAsDouble() * 600) / 2048; //real formula
    return (motor1.getVelocity().getValueAsDouble() * 600) / 2048;
  }

  public double getRPMfromVelocity2() {
    // return (motor2.getVelocity().getValueAsDouble() * 600) / 2048; //real formula
    return (motor2.getVelocity().getValueAsDouble()  * 600) / 2048;
  }

  public double RPMToVolts(double TargetRPM) {
    //Formula: VConstant = (AppliedVolts / VelocityatVolts) 
    double velocityConstant = 16 / 4500; //Sample Numbers -> vc = 0.002667
    return velocityConstant * TargetRPM;
  }


  public static double InchesToRPM(double inches) {
    return inches * 7.47 + 3800.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor 1 RPM", getRPMfromVelocity1());
    SmartDashboard.putNumber("Motor 2 RPM", getRPMfromVelocity2());
  }
}