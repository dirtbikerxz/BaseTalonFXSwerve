package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase {
    private TalonFX mShootMotor;
    public Shoot () {
        // To do: Initialize ShootMotor

    }
    public void shooterOn () {
        this.mShootMotor.set(1.0);
    }
    public void shooterOff () {
        this.mShootMotor.set(0.0);
    }
}
