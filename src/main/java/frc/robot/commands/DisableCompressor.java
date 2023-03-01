// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DisableCompressor extends CommandBase {
  private PneumaticHub pneumaticHub;



  /** Creates a new DisableCompressor. */
  public DisableCompressor() {
    pneumaticHub = new PneumaticHub(Constants.PNEUMATIC_HUB_ID);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(pneumaticHub.getCompressor()==true){
      pneumaticHub.disableCompressor();
      SmartDashboard.putString("Compressor", "DISABLED");
    }
    else{
      pneumaticHub.enableCompressorDigital();
      SmartDashboard.putString("Compressor", "");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
