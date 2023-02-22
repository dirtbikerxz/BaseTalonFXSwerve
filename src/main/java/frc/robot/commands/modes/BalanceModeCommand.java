package frc.robot.commands.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.XboxController; 
import frc.robot.Robot;
import frc.robot.commands.swerve.AlignToWallCommand;
import frc.robot.commands.swerve.ParkingOnThePlatformCommand;
import frc.robot.commands.swerve.SwerveCommands;

public class BalanceModeCommand extends SequentialCommandGroup {

    public BalanceModeCommand(Robot robot, XboxController inputController){
        addCommands(new AlignToWallCommand(robot, 0));
        addCommands(new ParkingOnThePlatformCommand(robot, inputController));
        addCommands(SwerveCommands.turnWheels(robot.swerveDrive, 90));
        addRequirements(robot.swerveDrive);
    }
}
