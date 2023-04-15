// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMode;
import frc.robot.RobotMode.ModeOptions;
import frc.robot.RobotMode.StateOptions;


public class Elevator extends SubsystemBase {
  
    public RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorMotor;
    private ProfiledPIDController controller;
    private ElevatorFeedforward ff;
    public double targetElevatorPosition = Constants.ELEVATOR_DEFUALT_STOW_LEVEL;

    /* Logging */
    private DataLog logger;

    // /* Elevator Motor */
    private DoubleLogEntry elevatorMotorTemperature;
    private DoubleLogEntry elevatorMotorAppliedOutput;
    private DoubleLogEntry elevatorMotorOutputCurrent;
    private IntegerLogEntry elevatorMotorFaults;

    // /* Elevator Motor Internal Encoder  */
    // private DoubleLogEntry elevatorEncoderPosition;
    // private DoubleLogEntry elevatorEncoderVelocity;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        //TODO: Set Current Limiters
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.FORWARD_ELEVATOR_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.REVERSE_ELEVATOR_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder(); 
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        /* Current Limits */
        elevatorMotor.setSmartCurrentLimit(40);

        /*Rate of Speed (Based on 930 Code) */

        // double p = SmartDashboard.getNumber("p", 0);
        this.controller = new ProfiledPIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D, new Constraints(80, 1000));
        this.controller.setTolerance(100, 100);
        // TODO: Recalculate these constants
        this.ff = new ElevatorFeedforward(0, Constants.ELEVATOR_G, 0.0, 0.0);

        elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);
        elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);

        // Create logger object 
        logger = DataLogManager.getLog();

        // // Create log entries for elevator motor
        elevatorMotorTemperature = new DoubleLogEntry(logger, "elevatorMotor/temperature");
        elevatorMotorAppliedOutput = new DoubleLogEntry(logger, "elevatorMotor/appliedOutput");
        elevatorMotorOutputCurrent = new DoubleLogEntry(logger, "elevatorMotor/outputCurrent");
        elevatorMotorFaults = new IntegerLogEntry(logger, "elevatorMotor/faults");

        // // Create log entries for elevator encoder
        // elevatorEncoderPosition = new DoubleLogEntry(logger, "elevatorEncoder/position");
        // elevatorEncoderVelocity = new DoubleLogEntry(logger, "elevatorEncoder/velocity");
    }
    
    


    /* Sets the Target Elevator Position in inches.*/
    public double getTargetElevatorPosition(){
        return targetElevatorPosition;
    }

    public void extend() {

        targetElevatorPosition = targetElevatorPosition + Constants.MANUAL_ELEVATOR_SPEED;

        if (targetElevatorPosition >= Constants.FORWARD_ELEVATOR_LIMIT) {
            targetElevatorPosition = Constants.FORWARD_ELEVATOR_LIMIT;
        }

    }

    public void retract() {

        targetElevatorPosition = targetElevatorPosition - Constants.MANUAL_ELEVATOR_SPEED;

        if (targetElevatorPosition <= Constants.REVERSE_ELEVATOR_LIMIT) {
            targetElevatorPosition = Constants.REVERSE_ELEVATOR_LIMIT;
        }
        
    }

    public void stop() {
        elevatorMotor.set(0);
    } 
    
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    private double motorRotationsToInches(double rotations) {
        return rotations * Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    private double inchesToMotorRotations(double inches) {
        return inches / Constants.ELEVATOR_ROTATIONS_TO_IN;
    }

    public boolean atPosition(double position) {

        double error = Math.abs(elevatorEncoder.getPosition() - position);

        if (Constants.ELEVATOR_TOLERANCE >= error) {
            return true;

        } else {
            return false;
        }
        
    }

    public Boolean isHigh(){
        return getTargetElevatorPosition() == Constants.ELEVATOR_DEFUALT_HIGH_LEVEL;
    }
    
    @Override
    public void periodic() {
        if (DriverStation.isEnabled()){
            // This method will be called once per scheduler run
            // TODO: Test that .getPosition() gives us the elevator position in inches
            
           // double feedforward = ff.calculate(/*double */elevatorEncoder.getVelocity());
            //System.out.println(feedforward);
            double voltage = controller.calculate(elevatorEncoder.getPosition(), targetElevatorPosition);

            MathUtil.clamp(voltage, -12, 12);

            elevatorMotor.setVoltage(voltage);

            
            SmartDashboard.putNumber("ELEVATOR PID VOLTAGE", voltage);
        }

        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("debug/Elevator Encoder Value: ", getEncoderPosition());
        
        logData();
    }


    /*
    public Command SetElevatorPosition (double inches){
        return new InstantCommand(() -> setTargetElevatorPosition(inches), this);
    }
    */

    public Command ElevatorAtPosition(double position){
        return Commands.waitUntil(() -> atPosition(position));
    }
    

    private void logData() {
        /* Elevator Motor */
        elevatorMotorTemperature.append(elevatorMotor.getMotorTemperature());
        elevatorMotorAppliedOutput.append(elevatorMotor.getAppliedOutput());
       // elevatorMotorBusVoltage.append(elevatorMotor.getBusVoltage());
        elevatorMotorOutputCurrent.append(elevatorMotor.getOutputCurrent());
      //  elevatorMotorClosedLoopRampRate.append(elevatorMotor.getClosedLoopRampRate());
    //    elevatorMotorOpenLoopRampRate.append(elevatorMotor.getOpenLoopRampRate());
        elevatorMotorFaults.append(elevatorMotor.getFaults());
    //    elevatorMotorIdleMode.append(elevatorMotor.getIdleMode().toString());
    //    elevatorMotorInverted.append(elevatorMotor.getInverted());
     //   elevatorMotorLastError.append(elevatorMotor.getLastError().toString());

        // /* Elevator Encoder */
        // elevatorEncoderPosition.append(getEncoderPosition());
        // elevatorEncoderVelocity.append(elevatorEncoder.getVelocity());
      }
}

