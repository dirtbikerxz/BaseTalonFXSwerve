// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.Constants.Swerve.IntakeState;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;

/** An example command that uses an example subsystem. */
public class TransferCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Transfer m_subsystem;
  private final Intake i_subsystem;
  private final DigitalInput transfer_command_input;
  private final DigitalInput transfer_stop_Input;
  private IntakeState intakePosition;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TransferCommand(Transfer subsystem, Intake intakeSystem) {
    m_subsystem = subsystem;
    i_subsystem = intakeSystem;
    intakePosition = IntakeState.Deactivated;
    transfer_command_input = intakeSystem.input;
    transfer_stop_Input = subsystem.transferInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_subsystem.runTransfer();
    if (transfer_command_input.get() && intakePosition == IntakeState.Deactivated) {
      i_subsystem.moveToTransfer();
      m_subsystem.runTransfer();
    };

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i_subsystem.stopMoveToTransfer();
    m_subsystem.stopTransfer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
