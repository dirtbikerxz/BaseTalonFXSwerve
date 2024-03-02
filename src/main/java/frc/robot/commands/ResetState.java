// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Spamp;

public class ResetState extends Command {
  private final Acquisition m_acquisition;
  private final Spamp m_spamp;
  /** Creates a new ClearIntake. */
  public ResetState(Acquisition subsystem1, Spamp subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_acquisition = subsystem1;
    addRequirements(m_acquisition);
    m_spamp = subsystem;
    addRequirements(m_spamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_acquisition.retractIntake();
    m_spamp.retractShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_acquisition.stopBoth();
    m_spamp.stopall();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
