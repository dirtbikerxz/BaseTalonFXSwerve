// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
  private ProfiledPIDController controller;
  private CANSparkMax armMotor;
  private CANCoder armEncoder;

  //private double targetArmAngle;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armEncoder = new CANCoder(Constants.ARM_ENCODER_ID);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  
  this.controller = new ProfiledPIDController(2, 0, 0, new Constraints(80, 1000));
  this.controller.setTolerance(1, 1);
        // TODO: Recalculate these constants
        armMotor = new CANSparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
  }

  public void stop() {
    armMotor.set(0);
  }

  private double armPositionInDegrees() {
    return getEncoderOffsetPosition() * Constants.encoderRatio;
  }
  //How many degrees the robot needs to rotate to get to Target Angle
  private double Netposition = Constants.targetArmAngle-armPositionInDegrees();

   //Sets the targetArmAngle in degrees */
   public void setTargetArmAngle(double degrees) {
     double targetArmAngle = degrees;
   }
  public double getEncoderAngle() {
    return armEncoder.getAbsolutePosition();
  }
//Gets the angle that the encoder is set to after taking care of the offset
  public double getEncoderOffsetPosition() {
    return getEncoderAngle()-Constants.ENCODER_OFFSET;
  }

  public void MoveArmPosition(){
    armMotor.set(0.25);{
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isEnabled()){
      // This method will be called once per scheduler run
      // TODO: Test that .getPosition() gives us the elevator position in inches
      double voltage = controller.calculate(armEncoder.getPosition(), getEncoderAngle());
      MathUtil.clamp(voltage, -12, 12);

      armMotor.setVoltage(voltage);
    }
  }
}
