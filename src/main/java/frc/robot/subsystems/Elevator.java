// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  
    public RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorMotor;
    private ProfiledPIDController controller;
    private ElevatorFeedforward ff;
    private double targetElevatorPosition;

    public Elevator() {
        elevatorMotor = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        //TODO: Set Current Limiters
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.FORWARD_ELEVATOR_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.REVERSE_ELEVATOR_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder(); 
        elevatorMotor.setIdleMode(IdleMode.kCoast);

        /*Rate of Speed (Based on 930 Code) */

        // double p = SmartDashboard.getNumber("p", 0);
        this.controller = new ProfiledPIDController(2, 0, 0, new Constraints(80, 1000));
        this.controller.setTolerance(1, 1);
        // TODO: Recalculate these constants
        //this.ff = new ElevatorFeedforward(0, 0.16, 0.18, 0.02);

        elevatorEncoder.setPositionConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);
        elevatorEncoder.setVelocityConversionFactor(Constants.ELEVATOR_ROTATIONS_TO_IN);
    }
    
    
    /* Sets the Target Elevator Position in inches.*/
    public void setTargetElevatorPosition(double inches){
        targetElevatorPosition = inches;
    }

    public void extend() {
        elevatorMotor.set(0.25);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    }

    public void retract() {
        elevatorMotor.set(-0.25);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    }

    public void stop() {
        elevatorMotor.set(0);
        SmartDashboard.putNumber("where_robotis", elevatorEncoder.getPosition());
    } 
    
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    private double motorRotationsToInches(double rotations) {
        return rotations * Constants.ELEVATOR_ROTATIONS_TO_IN;
    }
    
    @Override
    public void periodic() {
        if (DriverStation.isEnabled()){
            // targetElevatorPosition = 15;
            // This method will be called once per scheduler run
            // TODO: Test that .getPosition() gives us the elevator position in inches
            double voltage = controller.calculate(elevatorEncoder.getPosition(), targetElevatorPosition);
            // double feedforward = ff.calculate(/*double */elevatorEncoder.getVelocity());
            MathUtil.clamp(voltage, -12, 12);

            elevatorMotor.setVoltage(voltage);

            SmartDashboard.putNumber("ELEVATOR PID VOLTAGE", voltage);
        }
        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("Elevator Encoder Value: ", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Encoder Value (Inches): ", Units.metersToInches(elevatorEncoder.getPosition()));
        //SmartDashboard.putNumber("Elevator feedforward", feedforward);
    }
}

