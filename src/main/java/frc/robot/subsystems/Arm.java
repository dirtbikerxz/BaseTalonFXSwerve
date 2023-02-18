// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private ProfiledPIDController controller;
  private CANSparkMax armMotor;
  private CANCoder armEncoder;
  private double netPosition;
  private double targetArmAngle;

  //private double targetArmAngle;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = new CANCoder(Constants.ARM_ENCODER_ID);
    armMotor.setIdleMode(IdleMode.kBrake);
    //TODO:ADDBACKSOFTLIMIT
   armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
   armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  
    this.controller = new ProfiledPIDController(2, 0, 0, new Constraints(80, 1000));
    this.controller.setTolerance(1, 1);
    armEncoder.configFactoryDefault();
    armEncoder.configMagnetOffset(Constants.ARM_ENCODER_OFFSET);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  /* Always use this method when you want the position of the arm */
  private double getPositionInDegrees() {
    return armEncoder.getAbsolutePosition() * Constants.ARM_ENCODER_RATIO;
  }

  //Sets the targetArmAngle in degrees */
  public void setTargetArmAngle(double degrees) {
    targetArmAngle = degrees;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isEnabled()){
      // This method will be called once per scheduler run
      // TODO: Test that .getPosition() gives us the elevator position in inches
      // double voltage = controller.calculate(armEncoder.getPosition(), getEncoderAngle());
      // MathUtil.clamp(voltage, -12, 12);

      // armMotor.setVoltage(voltage);
    }
    SmartDashboard.putNumber(C, netPosition)
    if (armEncoder != null) {
      SmartDashboard.putNumber("Arm Position", getPositionInDegrees());    }
  }
}
