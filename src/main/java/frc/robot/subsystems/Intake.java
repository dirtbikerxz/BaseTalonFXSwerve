// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {

    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private DoubleSolenoid solenoid;
    PneumaticHub pneumaticHub;

    /* Logging */
    // private DataLog logger;

    // // TODO: Add logging for the solenoid
    // // TODO: Remove a motor when it has been determned which one is actually on the robot
    // /* Intake Motor 1 */
    // private DoubleLogEntry intakeMotor1Temperature;
    // private DoubleLogEntry intakeMotor1AppliedOutput;
    // private DoubleLogEntry intakeMotor1BusVoltage;
    // private DoubleLogEntry intakeMotor1OutputCurrent;
    // private DoubleLogEntry intakeMotor1ClosedLoopRampRate;
    // private DoubleLogEntry intakeMotor1OpenLoopRampRate;
    // private IntegerLogEntry intakeMotor1Faults;
    // private StringLogEntry intakeMotor1IdleMode;
    // private BooleanLogEntry intakeMotor1Inverted;
    // private StringLogEntry intakeMotor1LastError;

    // /* Intake Motor 2 */
    // private DoubleLogEntry intakeMotor2Temperature;
    // private DoubleLogEntry intakeMotor2AppliedOutput;
    // private DoubleLogEntry intakeMotor2BusVoltage;
    // private DoubleLogEntry intakeMotor2OutputCurrent;
    // private DoubleLogEntry intakeMotor2ClosedLoopRampRate;
    // private DoubleLogEntry intakeMotor2OpenLoopRampRate;
    // private IntegerLogEntry intakeMotor2Faults;
    // private StringLogEntry intakeMotor2IdleMode;
    // private BooleanLogEntry intakeMotor2Inverted;
    // private StringLogEntry intakeMotor2LastError;


    public Intake() {

      motor1 = new CANSparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);
      motor2 = new CANSparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);
      motor1.setIdleMode(IdleMode.kBrake);
      motor1.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
      motor2.enableSoftLimit(SoftLimitDirection.kForward, false);
      motor2.enableSoftLimit(SoftLimitDirection.kReverse, false);

      /* Current Limits */
      motor1.setSmartCurrentLimit(20);
      motor2.setSmartCurrentLimit(20);

      pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
      solenoid = pneumaticHub.makeDoubleSolenoid(Constants.PNEUMATIC_FORWARD_CHANNEL, Constants.PNEUMATIC_REVERSE_CHANEL);

      /* Logging */
      // logger = DataLogManager.getLog();

      // /* Intake Motor 1 */
      // intakeMotor1Temperature = new DoubleLogEntry(logger, "intake/motor1/temperature");
      // intakeMotor1AppliedOutput = new DoubleLogEntry(logger, "intake/motor1/appliedOutput");
      // intakeMotor1BusVoltage = new DoubleLogEntry(logger, "intake/motor1/busVoltage");
      // intakeMotor1OutputCurrent = new DoubleLogEntry(logger, "intake/motor1/outputCurrent");
      // intakeMotor1ClosedLoopRampRate = new DoubleLogEntry(logger, "intake/motor1/closedLoopRampRate");
      // intakeMotor1OpenLoopRampRate = new DoubleLogEntry(logger, "intake/motor1/openLoopRampRate");
      // intakeMotor1Faults = new IntegerLogEntry(logger, "intake/motor1/faults");
      // intakeMotor1IdleMode = new StringLogEntry(logger, "intake/motor1/idleMode");
      // intakeMotor1Inverted = new BooleanLogEntry(logger, "intake/motor1/inverted");
      // intakeMotor1LastError = new StringLogEntry(logger, "intake/motor1/lastError");
  
      // /* Intake Motor 2 */
      // intakeMotor2Temperature = new DoubleLogEntry(logger, "intake/motor2/temperature");
      // intakeMotor2AppliedOutput = new DoubleLogEntry(logger, "intake/motor2/appliedOutput");
      // intakeMotor2BusVoltage = new DoubleLogEntry(logger, "intake/motor2/busVoltage");
      // intakeMotor2OutputCurrent = new DoubleLogEntry(logger, "intake/motor2/outputCurrent");
      // intakeMotor2ClosedLoopRampRate = new DoubleLogEntry(logger, "intake/motor2/closedLoopRampRate");
      // intakeMotor2OpenLoopRampRate = new DoubleLogEntry(logger, "intake/motor2/openLoopRampRate");
      // intakeMotor2Faults = new IntegerLogEntry(logger, "intake/motor2/faults");
      // intakeMotor2IdleMode = new StringLogEntry(logger, "intake/motor2/idleMode");
      // intakeMotor2Inverted = new BooleanLogEntry(logger, "intake/motor2/inverted");
      // intakeMotor2LastError = new StringLogEntry(logger, "intake/motor2/lastError");

    }

    public void Run(double speed) {

      motor1.set(speed);
      motor2.set(-speed);

    }

    public void Stop() {

      motor1.set(0.15);
      motor2.set(0.15);

    }

    public void Extend() {

      solenoid.set(DoubleSolenoid.Value.kForward);

    }
    
    public void Retract() {

      solenoid.set(DoubleSolenoid.Value.kReverse);
      
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
      // intakeMotor1Temperature.append(motor1.getMotorTemperature());
      // intakeMotor1AppliedOutput.append(motor1.getAppliedOutput());
      // intakeMotor1BusVoltage.append(motor1.getBusVoltage());
      // intakeMotor1OutputCurrent.append(motor1.getOutputCurrent());
      // intakeMotor1ClosedLoopRampRate.append(motor1.getClosedLoopRampRate());
      // intakeMotor1OpenLoopRampRate.append(motor1.getOpenLoopRampRate());
      // intakeMotor1Faults.append(motor1.getFaults());
      // intakeMotor1IdleMode.append(motor1.getIdleMode().toString());
      // intakeMotor1Inverted.append(motor1.getInverted());
      // intakeMotor1LastError.append(motor1.getLastError().toString());

      // /* Intake Motor 2 */
      // intakeMotor2Temperature.append(motor2.getMotorTemperature());
      // intakeMotor2AppliedOutput.append(motor2.getAppliedOutput());
      // intakeMotor2BusVoltage.append(motor2.getBusVoltage());
      // intakeMotor2OutputCurrent.append(motor2.getOutputCurrent());
      // intakeMotor2ClosedLoopRampRate.append(motor2.getClosedLoopRampRate());
      // intakeMotor2OpenLoopRampRate.append(motor2.getOpenLoopRampRate());
      // intakeMotor2Faults.append(motor2.getFaults());
      // intakeMotor2IdleMode.append(motor2.getIdleMode().toString());
      // intakeMotor2Inverted.append(motor2.getInverted());
      // intakeMotor2LastError.append(motor2.getLastError().toString());
    }

}
