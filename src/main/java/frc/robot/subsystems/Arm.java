// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private ProfiledPIDController controller;
  private CANSparkMax armMotor;
  private CANCoder armCANEncoder;
  private ArmFeedforward ff;
  private double netPosition;
  private double targetArmAngle = Constants.ARM_STOW_POSITION;
  private double voltage;
  // public RelativeEncoder armRelativeEncoder;

  //private double targetArmAngle;

  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armCANEncoder = new CANCoder(Constants.ARM_ENCODER_ID);
    armMotor.setIdleMode(IdleMode.kBrake);
    this.ff = new ArmFeedforward(Constants.ARM_S, Constants.ARM_G, Constants.ARM_V, Constants.ARM_A);

  // armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  //armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
 // armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_FORWARD_LIMIT);
//armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_REVERSE_LIMIT);
  
    this.controller = new ProfiledPIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D, new Constraints(80, 1000));
    this.controller.setTolerance(1, 1);
    armCANEncoder.configFactoryDefault();
    armCANEncoder.configMagnetOffset(Constants.ARM_ENCODER_OFFSET);
    armCANEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    
    // armRelativeEncoder = armMotor.getEncoder();
    // armRelativeEncoder.setPosition(0.0);
    // armRelativeEncoder.setPositionConversionFactor(1.0);
    // armRelativeEncoder.setPosition(getPositionInDegrees());
    // armRelativeEncoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
    // armRelativeEncoder.setVelocityConversionFactor(Constants.ARM_GEAR_RATIO);
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  /* Always use this method when you want the position of the arm */
  private double getPositionInDegrees() {
    return armCANEncoder.getAbsolutePosition() / Constants.ARM_ENCODER_RATIO;
  }

    /* Always use this method when you want the velocity of the arm */
    private double getVelocityInDegrees() {
      return armCANEncoder.getVelocity() / Constants.ARM_ENCODER_RATIO;
    }


  //Sets the targetArmAngle in degrees */
  public void setTargetArmAngle(double degrees) {
    targetArmAngle = degrees;
  }

  public void moveArmUp() {
    
    targetArmAngle = targetArmAngle + Constants.MANUAL_ARM_SPEED;

    if (targetArmAngle >= Constants.FORWARD_ARM_LIMIT) {
        targetArmAngle = Constants.FORWARD_ARM_LIMIT;
    }

  }

  public void moveArmDown() {
    targetArmAngle = targetArmAngle - Constants.MANUAL_ARM_SPEED;

    if (targetArmAngle <= Constants.REVERSE_ARM_LIMIT) {
        targetArmAngle = Constants.REVERSE_ARM_LIMIT;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isEnabled()){
      // This method will be called once per scheduler run
      double effort = controller.calculate(getPositionInDegrees(), targetArmAngle); //TODO check directionality probably needs to be negated
      
      double feedforward = -ff.calculate(Units.degreesToRadians(getPositionInDegrees()), Units.degreesToRadians(getVelocityInDegrees())); //Negative due to gear between output and encoder reversing direction
      voltage =  feedforward; //TODO: add effort to incorporate profiled PID
      MathUtil.clamp(voltage, -12, 12);
      armMotor.setVoltage(voltage);

    }
    SmartDashboard.putNumber("CANCoder", armCANEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Position", getPositionInDegrees());
    SmartDashboard.putNumber("Arm Voltage", voltage);
    // SmartDashboard.putNumber("NEO (Relative) Encoder", armRelativeEncoder.getPosition());
  }

  private State getEncoderAngle() {
    return null;
  }
}
