// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends CommandBase {

  Swerve swerve;
  double Xdrive;
  double Ydrive;
  double rotation;

  /** Creates a new DriveForward. */
  public DriveCommand(Swerve swerve, double Xdrive, double Ydrive, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.Xdrive = Xdrive;
    this.Ydrive = Ydrive;
    this.rotation = rotation;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(Xdrive, Ydrive), rotation, false, true);
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
