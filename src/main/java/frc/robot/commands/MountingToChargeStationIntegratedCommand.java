package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.Robot;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.ParkingOnThePlatformCommand;
import frc.robot.commands.swerve.RotatingWheelsToADegreeCommand;
import edu.wpi.first.math.geometry.Rotation2d;

public class MountingToChargeStationIntegratedCommand extends SequentialCommandGroup{

    private final SwerveDriveSubsystem swerveDrive;
    private final XboxController inpuController;

    public MountingToChargeStationIntegratedCommand(Robot robot, XboxController inputController){

        this.swerveDrive = robot.swerveDrive;
        this.inpuController =inputController;
        
        addRequirements(robot.swerveDrive);

        addCommands(new AlignToWallCommand(robot, 0));
        addCommands(new ParkingOnThePlatformCommand(robot, inputController));
        addCommands(new RotatingWheelsToADegreeCommand(robot, 90));
    }

    
}
