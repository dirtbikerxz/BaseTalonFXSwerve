package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveTest extends SubsystemBase {

    private final CANSparkMax[] mDriveMotors = {
            new CANSparkMax(Constants.Swerve.Mod0.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod1.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod2.driveMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod3.driveMotorID, MotorType.kBrushless),
    };

    private final CANSparkMax[] mSteeringMotors = {
            new CANSparkMax(Constants.Swerve.Mod0.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod1.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod2.angleMotorID, MotorType.kBrushless),
            new CANSparkMax(Constants.Swerve.Mod3.angleMotorID, MotorType.kBrushless),
    };

    public void setDriveSpeed(int module, double speed) {
        if (module < 0 || module > 3) {
            return;
        }

        mDriveMotors[module].set(speed);
    }

    public void setSteeringSpeed(int module, double speed) {
        if (module < 0 || module > 3) {
            return;
        }

        mSteeringMotors[module].set(speed);
    }
}
