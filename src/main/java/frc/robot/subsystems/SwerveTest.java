package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.doubleNeoConstants;

import java.util.ArrayList;

public class SwerveTest extends SubsystemBase {

    private final ArrayList<TalonFX> mDriveMotors = new ArrayList<>();
    //  = {
    //         new TalonFX(doubleNeoConstants.Swerve.Mod0.driveMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod1.driveMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod2.driveMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod3.driveMotorID)
    // };

    private final ArrayList<TalonFX> mSteeringMotors = new ArrayList<>();
    //  = {
    //         new TalonFX(doubleNeoConstants.Swerve.Mod0.angleMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod1.angleMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod2.angleMotorID),
    //         new TalonFX(doubleNeoConstants.Swerve.Mod3.angleMotorID)
    // };

    public SwerveTest() {
        TalonFX drive0 = new TalonFX(doubleNeoConstants.Swerve.Mod0.driveMotorID);
        TalonFX drive1 = new TalonFX(doubleNeoConstants.Swerve.Mod1.driveMotorID);
        TalonFX drive2 = new TalonFX(doubleNeoConstants.Swerve.Mod2.driveMotorID);
        TalonFX drive3 = new TalonFX(doubleNeoConstants.Swerve.Mod3.driveMotorID);

        TalonFX steer0 = new TalonFX(doubleNeoConstants.Swerve.Mod0.angleMotorID);
        TalonFX steer1 = new TalonFX(doubleNeoConstants.Swerve.Mod1.angleMotorID);
        TalonFX steer2 = new TalonFX(doubleNeoConstants.Swerve.Mod2.angleMotorID);
        TalonFX steer3 = new TalonFX(doubleNeoConstants.Swerve.Mod3.angleMotorID);

        mDriveMotors.add(drive0);
        mDriveMotors.add(drive1);
        mDriveMotors.add(drive2);
        mDriveMotors.add(drive3);

        mSteeringMotors.add(steer0);
        mSteeringMotors.add(steer1);
        mSteeringMotors.add(steer2);
        mSteeringMotors.add(steer3);

    }

    public void setDriveSpeed(int module, double speed) {
        if (module < 0 || module > 3) {
            return;
        }

        mDriveMotors.get(module).set(speed);
    }

    public void setSteerPosition(int module, double position) {
        if (module < 0 || module > 3) {
            return;
        }

        mSteeringMotors.get(module).set(position);
    }
}
