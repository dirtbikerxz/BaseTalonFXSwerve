// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spamp;

public class autonAmp extends Command {
  private final Spamp m_spamp;

  public autonAmp(Spamp subsystem) {
    m_spamp = subsystem;
    addRequirements(m_spamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_spamp.runTopAmp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spamp.stopall();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_spamp.isNoteInSpamp() && m_spamp.isNoteLeavingSpamp(); 
  }
}
