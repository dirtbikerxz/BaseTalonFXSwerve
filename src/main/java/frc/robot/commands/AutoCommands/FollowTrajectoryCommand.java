// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class FollowTrajectoryCommand extends CommandBase {

    Swerve swerve;
    PathPlannerTrajectory traj;
    boolean isFirstPath;

    /** Creates a new DriveForward. */
    public FollowTrajectoryCommand(Swerve swerve, PathPlannerTrajectory traj, boolean isFirstPath) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.swerve = swerve;
      addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.followTrajectoryCommand(traj, isFirstPath);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}   
