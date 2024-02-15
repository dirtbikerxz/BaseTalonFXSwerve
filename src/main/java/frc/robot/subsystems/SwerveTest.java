package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.doubleNeoConstants;

public class SwerveTest extends SubsystemBase {

    private final TalonFX[] mDriveMotors = {
            new TalonFX(doubleNeoConstants.Swerve.Mod0.driveMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod1.driveMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod2.driveMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod3.driveMotorID)
    };

    private final TalonFX[] mSteeringMotors = {
            new TalonFX(doubleNeoConstants.Swerve.Mod0.angleMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod1.angleMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod2.angleMotorID),
            new TalonFX(doubleNeoConstants.Swerve.Mod3.angleMotorID)
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

        mSteeringMotors[module].setPosition(position);
    }
}
