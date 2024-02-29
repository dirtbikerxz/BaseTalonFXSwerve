package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Intake extends SubsystemBase {
    private final TalonSRX intakeController;
    private final double intakeSpeed;
    private final DoubleSolenoid m_doubleSolenoid;
    // private final Compressor m_compressor;

    public Intake() {
        intakeController = new TalonSRX(motorID);
        intakeSpeed = maxSpeed;
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
        
        //pressure switch actually turns off the pressurvizer at around 125-130 psi ????
        //gague might be bad, but it works 

        // m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        // m_compressor.enableAnalog(100, 115);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        //SmartDashboard.putBoolean("Compressor", m_compressor.isEnabled());
    }

    public void runIntake() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
        intakeController.set(ControlMode.PercentOutput, intakeSpeed);
    }

    public void toggleIntake() {
        m_doubleSolenoid.toggle();
    }


    public void reverseIntake() {
        intakeController.set(ControlMode.PercentOutput, -intakeSpeed);
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stopIntake() {
        intakeController.set(ControlMode.PercentOutput, 0);
    }
}
