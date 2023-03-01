// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class ConfirmScore extends CommandBase {
  Arm arm;
  Elevator elevator;
  /** Creates a new Confirm Score. */
  public ConfirmScore(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(this.arm);
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.isHigh()) {
      arm.setTargetArmAngle(Constants.ARM_MID_POSITION);
    } else {
        elevator.setTargetElevatorPosition(Constants.ELEVATOR_LOW_LEVEL);
    }
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
