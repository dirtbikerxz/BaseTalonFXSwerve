package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.doubleNeoConstants;

public class SwerveTest extends SubsystemBase {

    private final CANSparkMax[] mDriveMotors = {
            new CANSparkMax(doubleNeoConstants.Swerve.Mod0.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod1.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod2.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod3.driveMotorID, MotorType.kBrushless),
    };

    private final CANSparkMax[] mSteeringMotors = {
            new CANSparkMax(doubleNeoConstants.Swerve.Mod0.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod1.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod2.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(doubleNeoConstants.Swerve.Mod3.angleMotorID, MotorType.kBrushless),
    };

    public void setDriveSpeed(int module, double speed) {
        if (module < 0 || module > 3) {
            return;
        }

        mDriveMotors[module].set(speed);
    }

    public void setSteerPosition(int module, double position) {
        if (module < 0 || module > 3) {
            return;
        }

        mSteeringMotors[module].set(position);
//        mSteeringMotors[module].getPIDController().setReference(position, CANSparkBase.ControlType.kPosition);
    }
}
