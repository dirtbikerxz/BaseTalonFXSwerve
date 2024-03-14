// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hang;

/** An example command that uses an example subsystem. */
public class InstantHangExtension extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Hang m_subsystem;

  /**
   *
   *
   * @param subsystem The subsystem used by this command.
   */
  public InstantHangExtension(Hang subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runHang(-1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopHang();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_subsystem.leftEncoder <= -49 && m_subsystem.rightEncoder <= -49){
      return true;
    }
    return false;
  }


}
