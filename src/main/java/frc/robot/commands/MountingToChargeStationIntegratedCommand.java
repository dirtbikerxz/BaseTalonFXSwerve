package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.Robot;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.ParkingOnThePlatformCommand;

public class MountingToChargeStationIntegratedCommand extends SequentialCommandGroup{

    public MountingToChargeStationIntegratedCommand(Robot robot, XboxController inputController){
        addCommands(new AlignToWallCommand(robot, 0));
        addCommands(new ParkingOnThePlatformCommand(robot, inputController));
        addCommands(new RotatingWheelsToADegreeCommand(robot, 90));        
        addRequirements(robot.swerveDrive);
    }

    
}
