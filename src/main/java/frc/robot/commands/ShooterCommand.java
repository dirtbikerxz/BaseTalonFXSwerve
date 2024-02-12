// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  private final ShooterSubsystem shooterSubsytem;
  private double leftVoltage;
  private double rightVoltage;
 // private double kickerVoltage;
  public ShooterCommand(ShooterSubsystem shooterSubsytem, double leftVoltage, double rightVoltage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsytem = shooterSubsytem;
    this.leftVoltage = leftVoltage;
    this.rightVoltage = rightVoltage;
    //this.kickerVoltage = kickerVoltage;
    addRequirements(shooterSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ShooterCMD started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   shooterSubsytem.setShooterSpeed(leftVoltage, rightVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsytem.setShooterSpeed(0, 0);
    System.out.println("ShooterCMD stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
