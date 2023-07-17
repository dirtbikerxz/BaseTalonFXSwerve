// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.logging;
import frc.lib.util.loggingUtil.LogManager;
// import frc.lib.util.loggingUtil.LogManager;
import frc.robot.subsystems.swerve.falcon.CTRESwerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggingSubsystem extends SubsystemBase {
  private CTRESwerve s_Swerve;
  
  /** Creates a new LoggingSubsystem. */
  public LoggingSubsystem(CTRESwerve s_Swerve) {
    
    this.s_Swerve = s_Swerve;
  }
  public void updateSwerveLogs() {
    double[] actualStates = {
      s_Swerve.mSwerveMods[0].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[0].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[1].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[1].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[2].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[2].getState().speedMetersPerSecond,
      s_Swerve.mSwerveMods[3].getAngle().getDegrees(),
      s_Swerve.mSwerveMods[3].getState().speedMetersPerSecond
    };
    LogManager.addDoubleArray("Swerve/actual swerve states", actualStates);
    double[] desiredStates = {
      s_Swerve.mSwerveMods[0].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[0].getDesiredVelocity(),
      s_Swerve.mSwerveMods[1].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[1].getDesiredVelocity(),
      s_Swerve.mSwerveMods[2].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[2].getDesiredVelocity(),
      s_Swerve.mSwerveMods[3].getDesiredAngle().getDegrees(),
      s_Swerve.mSwerveMods[3].getDesiredVelocity()
    };
    LogManager.addDoubleArray("Swerve/Desired Swerve States", desiredStates);
  }

  @Override
  public void periodic() {
    updateSwerveLogs();
    
  }
}