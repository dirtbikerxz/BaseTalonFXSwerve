package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.IntakeState;

import static frc.robot.Constants.Intake.*;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Intake extends SubsystemBase {
    private final TalonSRX intakeController;
    private final double intakeSpeed;
    private final DoubleSolenoid m_doubleSolenoid;
    private final Compressor m_compressor;
    private final DigitalInput input;
    public IntakeState intakePosition;

    public Intake() {
        intakeController = new TalonSRX(motorID);
        intakeSpeed = maxSpeed;
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
        intakePosition = IntakeState.Deactivated;
        //TODO: change to correct sensor port
        input = new DigitalInput(0);
        
        //pressure switch actually turns off the pressurvizer at around 125-130 psi ????
        //gague might be bad, but it works 

        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.putBoolean("Compressor Running", m_compressor.isEnabled());
        
        if (input.get() && intakePosition == IntakeState.Activated)
        {
            new InstantCommand(() -> new Intake().reverseIntake());
            intakePosition = IntakeState.Deactivated;
        }
    } 

    public void runIntake() {
        if (intakePosition == IntakeState.Deactivated)
        {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
            intakeController.set(ControlMode.PercentOutput, intakeSpeed);
            intakePosition = IntakeState.Activated;
        }
    }

    //i dunno if it works so ill just comment it out for now
    // public void toggleIntake() {
    //     m_doubleSolenoid.toggle();
    // }


    public void reverseIntake() {
        if (intakePosition == IntakeState.Activated)
        {
            intakeController.set(ControlMode.PercentOutput, 0);
            m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            intakePosition = IntakeState.Deactivated;
        }
    }
}
