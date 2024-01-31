package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Interface containing behavior needed to control a swerve module.
 *
 * One swerve module is a steering motor and a drive motor working together.
 * The module includes the encoders for sensing the rotation of the steering
 * and driving motors, whether those encoders are included in the motor
 * controller or a distinct, standalone unit.
 */
public interface SwerveModule {
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);
    void resetToAbsolute();

    Rotation2d getRotation();
    SwerveModulePosition getPosition();
    SwerveModuleState getState();

    default void dashboardPeriodic() {}
}
