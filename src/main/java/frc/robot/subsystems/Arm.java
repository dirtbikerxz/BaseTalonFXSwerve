package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final CANSparkMax armFollowerMotor;

    private final SparkPIDController armPID;

    public Arm() {
        armMotor = new CANSparkMax(Constants.Arm.armMotor, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armMotor.getPIDController();
        configArmMotor();

        armFollowerMotor = new CANSparkMax(Constants.Arm.armFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configFollowerMotor();

    }

    public void setShoot() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredShooterAngle);

    }

    public void setIntake() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredIntakeAngle);
    }

    public boolean endConditionIntake() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredIntakeAngle);
        return armMotor.getEncoder().getPosition() > (angle.getDegrees() + 1) && armMotor.getEncoder().getPosition() < (angle.getDegrees() - 1);
    }

    public boolean endConditionShoot() {
        Rotation2d angle = new Rotation2d(Constants.Arm.desiredShooterAngle);
        return armMotor.getEncoder().getPosition() > (angle.getDegrees() + 1) && armMotor.getEncoder().getPosition() < (angle.getDegrees() - 1);
    }

    public void stopSet() {
        armMotor.stopMotor();
    }

    private void setAngle(Rotation2d angle) {
        armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }



    private void configArmMotor() {
        armPID.setOutputRange(Constants.Arm.minAngle, Constants.Arm.maxAngle);
    }

    private void configFollowerMotor() {
        armFollowerMotor.follow(armMotor, true);
    }
}
