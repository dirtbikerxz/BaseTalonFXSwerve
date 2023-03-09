// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotateToScoreButton extends CommandBase {
 private Swerve swerve;
 private ProfiledPIDController PID;
  /** Creates a new RotateToScoreButton. */
  public RotateToScoreButton(Swerve swerve) {
   this.swerve = swerve;
    addRequirements(this.swerve);
    
     PID = new ProfiledPIDController(
      Constants.ROTATE_TO_SCORE_BUTTON_KP, 
      Constants.ROTATE_TO_SCORE_BUTTON_KI, 
      Constants.ROTATE_TO_SCORE_BUTTON_KD, 
      new Constraints(Constants.ROTATE_TO_SCORE_BUTTON_ACCELERATION, Constants.ROTATE_TO_SCORE_BUTTON_VELOCITY)
    );

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
