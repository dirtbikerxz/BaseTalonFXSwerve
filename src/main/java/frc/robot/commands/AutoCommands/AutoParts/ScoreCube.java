// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoParts;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotMode;
import frc.robot.commands.ConfirmScore;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ReturnFromScoring;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.MovementCommands.GoToHigh;
import frc.robot.commands.MovementCommands.GoToMid;
import frc.robot.commands.MovementCommands.GoToStow;
import frc.robot.subsystems.*;

public class ScoreCube extends SequentialCommandGroup {

    private Elevator elevator;
    private Wrist wrist;
    private Intake intake;

    public ScoreCube(Elevator elevator, Wrist wrist, Intake intake) {

        // Use addRequirements() here to declare subsystem dependencies.
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;

        addRequirements(wrist, elevator, intake);

        addCommands(

          RobotMode.ChangeMode(RobotMode.ModeOptions.CUBE),
          new InstantCommand(() -> intake.Stop()),
          new InstantCommand(() -> wrist.SetWristPosition(Constants.WRIST_CUBE_NODE_SAFE)),
          new GoToMid(wrist, elevator),
          new ReverseIntake(intake).withTimeout(0.5),
          new GoToStow(wrist, elevator)

        );
    }

}  