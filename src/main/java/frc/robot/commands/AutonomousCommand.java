package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.commands.arm.OldArmPresetCommand;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    private final SwerveDriveSubsystem swerveDrive;

    
    public AutonomousCommand(Robot robot){

        this.swerveDrive = robot.swerveDrive;

        robot.swerveDrive.zeroGyro();

        ParallelCommandGroup g = new ParallelCommandGroup(
            new AlignToWallCommand(robot, 0)
            // new OldArmPresetCommand(robot.arm, OldArmPresetCommand.TRAVEL_PRESET)
        );


        addCommands(g);
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0.8128, 0, 0), 3));
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0, 0, Units.degreesToRadians(-30)), 3));
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0.8128, 0, 0), 3));
        

    

        addRequirements(robot.swerveDrive);

    }




    
}
