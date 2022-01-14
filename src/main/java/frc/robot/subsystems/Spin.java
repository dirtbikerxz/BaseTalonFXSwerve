package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
// import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// michael frere was here

public class Spin extends SubsystemBase {

    Timer m_Timer = new Timer();

    private TalonFX randomMotor = new TalonFX(5);

    @Override
    public void periodic(){
    }

    public void initialize(){
        m_Timer.start();
    }

    public void spin(){
        randomMotor.set(ControlMode.PercentOutput, .5);
    }

    public void stop(){
        randomMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished(){
        if(m_Timer.get() > 3){
            return true;
        } else {
            return false;
        }
    }
}