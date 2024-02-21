package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class Intake extends SubsystemBase {
    private final TalonSRX intakeController;
    private final double intakeSpeed;
    private final DoubleSolenoid m_doubleSolenoid;

    public Intake() {
        intakeController = new TalonSRX(motorID);
        intakeSpeed = maxSpeed;
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
    }

    public void runIntake() {
        intakeController.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void toggleIntake() {
        m_doubleSolenoid.toggle();
    }


    public void reverseIntake() {
        intakeController.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    public void stopIntake() {
        intakeController.set(ControlMode.PercentOutput, 0);
    }
}
