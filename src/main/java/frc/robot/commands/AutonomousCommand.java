package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.commands.swerve.SwerveFixedSpeedCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is where all the logic for our autonomous program will go.
 */
public class AutonomousCommand extends SequentialCommandGroup {

    private final SwerveDriveSubsystem swerveDrive;

    
    public AutonomousCommand(Robot robot){

        this.swerveDrive = robot.swerveDrive;

        robot.swerveDrive.zeroGyro();
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0.8128, 0, 0), 3));
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0, 0, Units.degreesToRadians(-30)), 3));
        addCommands(new SwerveFixedSpeedCommand(swerveDrive, new ChassisSpeeds(0.8128, 0, 0), 3));
        

    

        addRequirements(robot.swerveDrive);

    }




    
}
