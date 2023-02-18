// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArmManual extends CommandBase {
  Arm arm;
  Joystick driver;
  /** Creates a new MoveArmManual. */
  public MoveArmManual(Arm arm, Joystick driver) {
    this.arm = arm;
    this.driver = driver;
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driver.getRawButton(3)) {
      arm.setSpeed(0.25);
    } else if (driver.getRawButton(2)) {
      arm.setSpeed(-0.25);
    } else {
      arm.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
