// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
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
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public RelativeEncoder armRelativeEncoder;
  public ProfiledPIDController controller;

  private CANSparkMax armMotor;
  private CANCoder armCANEncoder;
  private ArmFeedforward ff;
  private double netPosition;
  private double targetArmAngle = Constants.ARM_STOW_POSITION; //TODO: make not have initialization issue
  private double voltage;

  // Logging objects
  private DataLog logger;

  /* Arm Motor Logging */
  private DoubleLogEntry armMotorTemperature;
  private DoubleLogEntry armMotorAppliedOutput;
  private DoubleLogEntry armMotorBusVoltage;
  private DoubleLogEntry armMotorOutputCurrent;
  private DoubleLogEntry armMotorClosedLoopRampRate;
  private DoubleLogEntry armMotorOpenLoopRampRate;
  private IntegerLogEntry armMotorFaults;
  private StringLogEntry armMotorIdleMode;
  private BooleanLogEntry armMotorInverted;
  private StringLogEntry armMotorLastError;

  /* Arm CANCoder Logging */
  private DoubleLogEntry armCANCoderAbsolutePosition;
  private DoubleLogEntry armCANCoderAbsoluteVelocity;
  private DoubleLogEntry armCANCoderBusVoltage;


  /** Creates a new Arm. */
  public Arm() {

    armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
    armCANEncoder = new CANCoder(Constants.ARM_ENCODER_ID);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(true);
    this.ff = new ArmFeedforward(Constants.ARM_S, Constants.ARM_G, Constants.ARM_V, Constants.ARM_A);

    armMotor.setSmartCurrentLimit(40);

  // armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  //armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
 // armMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ARM_FORWARD_LIMIT);
//armMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.ARM_REVERSE_LIMIT);
  
    this.controller = new ProfiledPIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D, new Constraints(800, 1000));
    this.controller.setTolerance(1, 1);
    armCANEncoder.configFactoryDefault();
    armCANEncoder.configMagnetOffset(Constants.ARM_ENCODER_OFFSET);
    armCANEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    armCANEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    
    armRelativeEncoder = armMotor.getEncoder();
    //armRelativeEncoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
    armRelativeEncoder.setPositionConversionFactor(Constants.ARM_MOTOR_ROT_TO_DEG);
    armRelativeEncoder.setVelocityConversionFactor(Constants.ARM_MOTOR_ROT_TO_DEG);

    armRelativeEncoder.setPosition(getPositionInDegreesCanCoder() / Constants.ARM_MOTOR_ROT_TO_DEG);
    armRelativeEncoder.setPositionConversionFactor(1);
    armRelativeEncoder.setVelocityConversionFactor(Constants.ARM_GEAR_RATIO);

    // Create logger object 
    logger = DataLogManager.getLog();
    armMotorTemperature = new DoubleLogEntry(logger, "armMotor/temperature");
    armMotorAppliedOutput = new DoubleLogEntry(logger, "armMotor/appliedOutput");
    armMotorBusVoltage = new DoubleLogEntry(logger, "armMotor/busVoltage");
    armMotorOutputCurrent = new DoubleLogEntry(logger, "armMotor/outputCurrent");
    armMotorClosedLoopRampRate = new DoubleLogEntry(logger, "armMotor/closedLoopRampRate");
    armMotorOpenLoopRampRate = new DoubleLogEntry(logger, "armMotor/openLoopRampRate");
    armMotorFaults = new IntegerLogEntry(logger, "armMotor/faults");
    armMotorIdleMode = new StringLogEntry(logger, "armMotor/idleMode");
    armMotorInverted = new BooleanLogEntry(logger, "armMotor/inverted");
    armMotorLastError = new StringLogEntry(logger, "armMotor/lastError");
    armCANCoderAbsolutePosition = new DoubleLogEntry(logger, "armCANCoder/position");
    armCANCoderAbsoluteVelocity = new DoubleLogEntry(logger, "armCANCoder/velocity");
    armCANCoderBusVoltage = new DoubleLogEntry(logger, "armCANCoder/busVoltage");
  }

  public void resetRelative() {

    armRelativeEncoder.setPosition(getPositionInDegreesCanCoder());
  }

  public void setSpeed(double speed){
    armMotor.set(speed);
  }

  /* Always use this method when you want the position of the arm */
  public double getPositionInDegreesCanCoder() {
    return armCANEncoder.getAbsolutePosition() / Constants.ARM_ENCODER_RATIO;
  }

  public double getPositionInDegreesIntegrated() {
    return armRelativeEncoder.getPosition();
  }

  public double getVelocityInDegreesIntegrated() {
    return armCANEncoder.getVelocity();
  }

  /* Always use this method when you want the velocity of the arm */
  public double getVelocityInDegreesCanCoder() {
    return armCANEncoder.getVelocity() / Constants.ARM_ENCODER_RATIO;
  }

  //Sets the targetArmAngle in degrees */
  public void setTargetArmAngle(double degrees) {
    targetArmAngle = degrees;
  }

  public void moveArmUp() {
    
    targetArmAngle = targetArmAngle + Constants.MANUAL_ARM_SPEED;

    if (targetArmAngle >= Constants.ARM_FORWARD_LIMIT) {
        targetArmAngle = Constants.ARM_FORWARD_LIMIT;
    }

  }

  public void moveArmDown() {
    targetArmAngle = targetArmAngle - Constants.MANUAL_ARM_SPEED;

    if (targetArmAngle <= Constants.ARM_REVERSE_LIMIT) {
        targetArmAngle = Constants.ARM_REVERSE_LIMIT;
    }
  }

  public boolean atPosition() {

    double error = Math.abs(Math.abs(getPositionInDegreesCanCoder()) - Math.abs(targetArmAngle));

    if (Constants.ARM_TOLERANCE >= error) {

        return true;

    } else {
        return false;
    }
    
  }

  public boolean isSafeToGround() {
    return getPositionInDegreesCanCoder() < Constants.ARM_REVERSE_LIMIT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pid = controller.calculate(getPositionInDegreesCanCoder(), targetArmAngle);
    if (DriverStation.isEnabled()){      
      double feedForward = ff.calculate(Units.degreesToRadians(getPositionInDegreesCanCoder()), Units.degreesToRadians(getVelocityInDegreesCanCoder()));
      voltage =  pid + feedForward;

      MathUtil.clamp(voltage, -12, 12);
      armMotor.setVoltage(voltage);
      // SmartDashboard.putNumber("PID VALUE", pid);

      
    }

    /* Smart Dashboard printing */
    SmartDashboard.putNumber("CANCoder", armCANEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Position Integrated", getPositionInDegreesIntegrated());
    SmartDashboard.putNumber("Arm Position Absolute", getPositionInDegreesCanCoder());
    SmartDashboard.putNumber("Arm PID Output", pid);
    // SmartDashboard.putNumber("Arm Voltage", voltage);
    SmartDashboard.putNumber("Target Arm Angle", targetArmAngle);
    SmartDashboard.putNumber("Error", Math.abs(Math.abs(getPositionInDegreesCanCoder()) - Math.abs(targetArmAngle)));
    SmartDashboard.putString("CANCoder Initialization Strategy", armCANEncoder.configGetSensorInitializationStrategy().toString());
    
    //SmartDashboard.putNumber("NEO (Relative) Encoder", armRelativeEncoder.getPosition());

    SmartDashboard.putNumber("Integrated Encoder", getPositionInDegreesIntegrated());

    // Logs all relevant data
    // Any additional data logging should be done in this method
    logData();
  }

  public Command SetArmPosition (double degrees){
    return new InstantCommand(() -> setTargetArmAngle(degrees), this);
  }

  public Command ArmAtPosition(){
    return Commands.waitUntil(() -> atPosition());
  }

  private State getEncoderAngle() {
    return null;
  }

  private void logData() {
    /* Arm Motor */
    // armMotorTemperature.append(armMotor.getMotorTemperature());
    // armMotorAppliedOutput.append(armMotor.getAppliedOutput());
    // armMotorBusVoltage.append(armMotor.getBusVoltage());
    // armMotorOutputCurrent.append(armMotor.getOutputCurrent());
    // armMotorClosedLoopRampRate.append(armMotor.getClosedLoopRampRate());
    // armMotorOpenLoopRampRate.append(armMotor.getOpenLoopRampRate());
    // armMotorFaults.append(armMotor.getFaults());
    // armMotorIdleMode.append(armMotor.getIdleMode().toString());
    // armMotorInverted.append(armMotor.getInverted());
    // armMotorLastError.append(armMotor.getLastError().toString());

    // /* Arm CANCoder */
    // armCANCoderAbsolutePosition.append(getPositionInDegreesCanCoder());
    // armCANCoderAbsoluteVelocity.append(getVelocityInDegreesCanCoder());
    // armCANCoderBusVoltage.append(armCANEncoder.getBusVoltage());
  }
}
