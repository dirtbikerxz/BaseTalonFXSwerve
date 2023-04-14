// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMode;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.xml.crypto.Data;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {

    private CANSparkMax motor;

    /* Logging */
    private DataLog logger;

    // TODO: Add logging for the solenoid
    // TODO: Remove a motor when it has been determned which one is actually on the robot
    /* Intake Motor */
    private DoubleLogEntry intakeMotor1Temperature;
    private DoubleLogEntry intakeMotor1AppliedOutput;
    private IntegerLogEntry intakeMotor1Faults;



    public Intake() {

      motor = new CANSparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
      motor.setIdleMode(IdleMode.kBrake);
      motor.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

      /* Current Limits */
      motor.setSmartCurrentLimit(20);

      /* Logging */
      logger = DataLogManager.getLog();

      /* Intake Motor */
      intakeMotor1Temperature = new DoubleLogEntry(logger, "intake/motor1/temperature");
      intakeMotor1AppliedOutput = new DoubleLogEntry(logger, "intake/motor1/appliedOutput");
      intakeMotor1Faults = new IntegerLogEntry(logger, "intake/motor1/faults");

    }

    public void Run(double speed) {

      motor.set(speed);

    }

    public void Stop() {

      if (RobotMode.mode == RobotMode.ModeOptions.CONE) {
        motor.set(0.15);
      } else {
        motor.set(-0.15);
      }

    }


    @Override
    public void periodic() {
      logData();
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    private void logData() {
      /* Intake Motor 1 */
      intakeMotor1Temperature.append(motor.getMotorTemperature());
      intakeMotor1AppliedOutput.append(motor.getAppliedOutput());
      //intakeMotor1BusVoltage.append(motor1.getBusVoltage());
      //intakeMotor1OutputCurrent.append(motor1.getOutputCurrent());
     // intakeMotor1ClosedLoopRampRate.append(motor1.getClosedLoopRampRate());
     // intakeMotor1OpenLoopRampRate.append(motor1.getOpenLoopRampRate());
      intakeMotor1Faults.append(motor.getFaults());
     // intakeMotor1IdleMode.append(motor1.getIdleMode().toString());
     // intakeMotor1Inverted.append(motor1.getInverted());
     // intakeMotor1LastError.append(motor1.getLastError().toString());
    }

}
