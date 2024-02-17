package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Hang.*;

public class Hang extends SubsystemBase {
    private final CANSparkMax leftHangController;
    private final CANSparkMax rightHangController;
    private final double hangSpeed;

    public Hang() {
        leftHangController = new CANSparkMax(leftHangID, MotorType.kBrushless); 
        rightHangController = new CANSparkMax(rightHangID, MotorType.kBrushless); 
        hangSpeed = maxSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Hang Current", leftHangController.getOutputCurrent());
    }

    public void runHang(int input) {
        //Positive is up on the right side and down on the left side
        leftHangController.set(-maxSpeed * input);
        rightHangController.set(maxSpeed * input);
    }

    public void stopHang() {
        leftHangController.set(0);
        rightHangController.set(0);
    }
}
