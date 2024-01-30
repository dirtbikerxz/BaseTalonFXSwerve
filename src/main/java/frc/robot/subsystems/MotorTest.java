package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorTest extends SubsystemBase {

    private CANSparkMax mod0Drive = new CANSparkMax(Constants.Swerve.Mod0.driveMotorID, MotorType.kBrushless);
    private CANSparkMax mod0Angle = new CANSparkMax(Constants.Swerve.Mod0.angleMotorID, MotorType.kBrushless);

    private CANSparkMax mod1Drive = new CANSparkMax(Constants.Swerve.Mod1.driveMotorID, MotorType.kBrushless);
    private CANSparkMax mod1Angle = new CANSparkMax(Constants.Swerve.Mod1.angleMotorID, MotorType.kBrushless);

    private CANSparkMax mod2Drive = new CANSparkMax(Constants.Swerve.Mod2.driveMotorID, MotorType.kBrushless);
    private CANSparkMax mod2Angle = new CANSparkMax(Constants.Swerve.Mod2.angleMotorID, MotorType.kBrushless);

    private CANSparkMax mod3Drive = new CANSparkMax(Constants.Swerve.Mod3.driveMotorID, MotorType.kBrushless);
    private CANSparkMax mod3Angle = new CANSparkMax(Constants.Swerve.Mod3.angleMotorID, MotorType.kBrushless);
    

    public MotorTest () {
        // To do: Initialize ShootMotor


    }

    public void driveMotor0(double speed){
        mod0Drive.set(speed);
    }
    public void angleMotor0(double speed){
        mod0Angle.set(speed);
    }

    public void driveMotor1(double speed){
        mod1Drive.set(speed);
    }
    public void angleMotor1(double speed){
        mod1Angle.set(speed);
    }

    public void driveMotor2(double speed){
        mod2Drive.set(speed);
    }
    public void angleMotor2(double speed){
        mod2Angle.set(speed);
    }

    public void driveMotor3(double speed){
        mod3Drive.set(speed);
    }
    public void angleMotor3(double speed){
        mod3Angle.set(speed);
    }

}
