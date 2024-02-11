package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterMotor;
    private final CANSparkFlex shooterFollowerMotor;
    private final CANSparkMax shooterIntakeMotor;

    private final SparkPIDController shooterPID;
    private final SparkPIDController shooterIntakePID;

    public Shooter() {
        shooterMotor = new CANSparkFlex(Constants.Shooter.shooterMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterPID = shooterMotor.getPIDController();
        configShooterMotor();

        shooterIntakeMotor = new CANSparkMax(Constants.Shooter.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterIntakePID = shooterIntakeMotor.getPIDController();
        configShooterIntakeMotor();

        shooterFollowerMotor = new CANSparkFlex(Constants.Shooter.shooterFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterFollowerMotor();

    }

    public void indexNote() {
        shooterIntakePID.setReference(Constants.Shooter.shooterIntakeSpeed, CANSparkBase.ControlType.kVelocity);
    }

    public void shootNote() {
        shooterPID.setReference(Constants.Shooter.shooterSpeed, CANSparkBase.ControlType.kVelocity);
    }

    public void stop() {
        shooterMotor.stopMotor();
    }

    private void configShooterMotor() {

    }

    private void configShooterFollowerMotor() {
        shooterFollowerMotor.follow(shooterMotor);
    }

    private void configShooterIntakeMotor() {

    }
}
