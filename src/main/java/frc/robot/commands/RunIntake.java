/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  
    private Intake intake;

    public RunIntake(Intake intake) {

        this.intake = intake;

        addRequirements(intake);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        intake.Run(Constants.INTAKE_SPEED);
        SmartDashboard.putBoolean("driver/ Intake Direction", true);
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        intake.Stop();
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        //neo.StopMotor();

        return false;
    }
}