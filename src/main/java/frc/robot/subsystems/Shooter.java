package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.Pigeon2Configuration;
// import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final TalonSRX shooterControllerL;
    private final TalonSRX shooterControllerR;
    private final double speed;

    public Shooter() {
        shooterControllerL = new TalonSRX(1); // TODO: change to correct port #
        shooterControllerR = new TalonSRX(2); // TODO: change to correct port #
        speed = .8;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter Speed", speed);
    }


    public void runShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, -speed);
        shooterControllerR.set(ControlMode.PercentOutput, speed);
    }

    public void stopShooter() {
        shooterControllerL.set(ControlMode.PercentOutput, 0);
        shooterControllerR.set(ControlMode.PercentOutput, 0);
    }

}