package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterControllerL;
    private final TalonSRX shooterControllerR;

    public Shooter() {
        shooterControllerL = new TalonSRX(1); // TODO: change to correct port #
        shooterControllerR = new TalonSRX(2); // TODO: change to correct port #
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Speed", Constants.Shooter.speed);
    }


    public void runShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, -Constants.Shooter.speed);
        shooterControllerR.set(ControlMode.PercentOutput, Constants.Shooter.speed);
    }

    public void stopShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, 0);
        shooterControllerR.set(ControlMode.PercentOutput, 0);
    }

}