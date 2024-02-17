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
    private double leftEncoder = 0;
    private double rightEncoder = 0;
    private double leftUpperLimit = 0; //TODO: find limit
    private double rightUpperLimit = 0; //TODO: find limit
    private double leftLowerLimit = 0; //TODO: find limit
    private double rightLowerLimit = 0; //TODO: find limit
    private boolean limits = false; //set to 'true' once the limits are done

    public Hang() {
        leftHangController = new CANSparkMax(leftHangID, MotorType.kBrushless); 
        rightHangController = new CANSparkMax(rightHangID, MotorType.kBrushless); 
        hangSpeed = maxSpeed;
    }

    @Override
    public void periodic() {
        leftEncoder =  leftHangController.getEncoder().getPosition();
        rightEncoder = rightHangController.getEncoder().getPosition();
        SmartDashboard.putNumber("Left Hang Current", leftHangController.getOutputCurrent());
        SmartDashboard.putNumber("Right Hang Current", rightHangController.getOutputCurrent());
        SmartDashboard.putNumber("Left Encoder", leftEncoder);
        SmartDashboard.putNumber("Right Encoder", rightEncoder);
    }

    public void runHang(int input) {    
        if(leftEncoder > leftLowerLimit && rightEncoder > rightLowerLimit && leftEncoder < leftUpperLimit && rightEncoder < rightUpperLimit && limits) {
            leftHangController.set(maxSpeed * input);
            rightHangController.set(maxSpeed * input);
        }
    }

    public void stopHang() {
        leftHangController.set(0);
        rightHangController.set(0);
    }

    public void removeHangLimits() { limits = false; }
}
