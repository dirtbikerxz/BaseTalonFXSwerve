// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class KickerCommand extends Command {
  /** Creates a new KickerCommand. */
  private final KickerSubsystem kickerSubsystem;
  private double kickerVoltage;
  public KickerCommand(KickerSubsystem kickerSubsystem, double kickerVoltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.kickerSubsystem = kickerSubsystem;
    this.kickerVoltage = kickerVoltage;
    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("kickerCMD started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kickerSubsystem.setKickerSpeed( kickerVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.setKickerSpeed(0);
    System.out.println("kickerCMD stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
