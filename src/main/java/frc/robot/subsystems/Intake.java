package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private final TalonSRX intakeController;
    private final double intakeSpeed;

    public Intake() {
        intakeController = new TalonSRX(motorID);
        intakeSpeed = maxSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
    }

    public void runIntake() {
        intakeController.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void reverseIntake() {
        intakeController.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void stopIntake() {
        intakeController.set(ControlMode.PercentOutput, 0);
    }
}
