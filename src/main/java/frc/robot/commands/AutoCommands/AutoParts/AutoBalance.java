// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoParts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoBalance extends CommandBase {

  Swerve swerve;
  PIDController pid;
  Alliance alliance;
  double balanceVar;

  /** Creates a new DriveForward. */
  public AutoBalance(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;

    pid = new PIDController(Constants.AUTO_BALANCE_P, Constants.AUTO_BALANCE_I, Constants.AUTO_BALANCE_D);
    addRequirements(this.swerve);
    alliance = DriverStation.getAlliance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (alliance == Alliance.Red) {

      balanceVar = Constants.RED_BALANCE_LEVEL;

    } else if (alliance == Alliance.Blue) {

      balanceVar = Constants.BLUE_BALANCE_LEVEL;

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angle = swerve.getRoll().getDegrees();
    // If we substract Constants.BALANCE_LEVEL that should make our PIDController treat Constants.BALANCE_LEVEL as level

    double pidVar = pid.calculate(angle - balanceVar);
    Translation2d move = new Translation2d(-pidVar, 0.0);

    if (angle <= balanceVar + Constants.BALANCE_LEVEL_DEADZONE && angle >= balanceVar - Constants.BALANCE_LEVEL_DEADZONE) {

      
      System.out.println("balanced");
      // new LockWheels(swerve);
      // swerve.lockWheels();
      swerve.drive(new Translation2d(0.0, 0.0), 0.5, true, false);
      swerve.drive(new Translation2d(0.0, 0.0), 0.0, true, false);

    } else {

      System.out.println("balancing");
      swerve.drive(move, 0.0, true, false);

    }

    SmartDashboard.putNumber("pidVar", pidVar);
    SmartDashboard.putNumber("Robot Angle", angle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0,0.0), 0.0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
