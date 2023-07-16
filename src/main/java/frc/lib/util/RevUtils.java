package frc.lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class RevUtils {

  /**
   *  Sets the drive motor configuration
   * 
   * 
   * @param motorController the motor controller to tune
   * @param smartDashboardTuning make this true in order to tune PID values in smart dashboard
   * 
   */
  public static void setDriveMotorConfig(CANSparkMax motorController, boolean smartDashboardTuning) {

    if (smartDashboardTuning) {
      SmartDashboard.putNumber("Drive FF", 0);
      SmartDashboard.putNumber("Drive P", 0);
      SmartDashboard.putNumber("Drive I", 0);
      SmartDashboard.putNumber("Drive D", 0);
      SmartDashboard.putNumber("Drive Ramp Rate", 0);

      motorController.getPIDController().setFF(SmartDashboard.getNumber("Drive FF", 0));
      motorController.getPIDController().setP(SmartDashboard.getNumber("Drive P", 0));
      motorController.getPIDController().setI(SmartDashboard.getNumber("Drive I", 0));
      motorController.getPIDController().setD(SmartDashboard.getNumber("Drive D", 0));

      motorController.setOpenLoopRampRate(SmartDashboard.getNumber("Drive Ramp Rate", 0));
    } else {
      motorController.getPIDController().setFF(0.15751);
      motorController.getPIDController().setP(0.23983);
      motorController.getPIDController().setI(0.0);
      motorController.getPIDController().setD(0.0);

      motorController.setOpenLoopRampRate(0.1);
    }

    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 50);

    motorController.setSmartCurrentLimit(60, 35);
  }

  /**
   *  Sets the turn motor configuration
   * 
   * 
   * @param motorController the motor controller to tune
   * @param smartDashboardTuning make this true in order to tune PID values in smart dashboard
   * 
   */
  public static void setTurnMotorConfig(CANSparkMax motorController, boolean smartDashboardTuning) {

    if (smartDashboardTuning) {
      SmartDashboard.putNumber("Turn FF", 0);
      SmartDashboard.putNumber("Turn P", 0);
      SmartDashboard.putNumber("Turn I", 0);
      SmartDashboard.putNumber("Turn D", 0);

      motorController.getPIDController().setFF(SmartDashboard.getNumber("Turn FF", 0));
      motorController.getPIDController().setP(SmartDashboard.getNumber("Turn P", 0));
      motorController.getPIDController().setI(SmartDashboard.getNumber("Turn I", 0));
      motorController.getPIDController().setD(SmartDashboard.getNumber("Turn D", 0));
    } else {
      motorController.getPIDController().setFF(0);
      motorController.getPIDController().setP(0.01);
      motorController.getPIDController().setI(0.0);
      motorController.getPIDController().setD(0.0);
    }

    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motorController.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);

    motorController.setSmartCurrentLimit(40, 25);
  }

  public static SwerveModuleState optimize(
          SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
            placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

}
