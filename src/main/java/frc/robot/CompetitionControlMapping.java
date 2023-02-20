package frc.robot;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HandCommands;
import frc.robot.commands.swerve.AlignToAprilTagCommand;
import frc.robot.commands.swerve.SetRobotRelativeCommand;
import frc.robot.commands.swerve.SwerveOrbitCommand;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.commands.swerve.TurboModeCommand;
import frc.robot.commands.swerve.ZeroGyroCommand;


public class CompetitionControlMapping {



    public static void mapDriverJoystick(Robot robot, CommandXboxController driveController) {
        
        // Align to April Tag
        driveController.a()
                .onTrue(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));    
        
        // Zero Gryo
        driveController.start()
            .onTrue(new ZeroGyroCommand(robot.swerveDrive));

        // Orbit Command
        driveController.rightTrigger(0.5)
        .onTrue(new SwerveOrbitCommand(robot.swerveDrive, true))
        .onFalse(new SwerveOrbitCommand(robot.swerveDrive, false));

        // Trubo Mode
        driveController.rightBumper()
            .onTrue(new TurboModeCommand(robot.swerveDrive, true))
            .onFalse(new TurboModeCommand(robot.swerveDrive, false));

        // Robot Relative
        driveController.leftBumper()
        .onTrue(new SetRobotRelativeCommand(robot.swerveDrive, true))
        .onFalse(new SetRobotRelativeCommand(robot.swerveDrive, false));
            
    }

    public static void mapSpecialOpsJoystick(Robot robot, CommandXboxController specialOpsController) {
        
        //TODO add the commands for the arm hights.    
        specialOpsController.pov(0) // High
            .onTrue(null); 
        specialOpsController.pov(90)
            .onTrue(null); // Middle Hight
        specialOpsController.pov(180)
            .onTrue(null); // Low
        specialOpsController.pov(270)
            .onTrue(null); // Travel
            
        // Hand grabbing commands
        specialOpsController.y()
        .onTrue(new HandCommands().grabCone(robot.hand));

        specialOpsController.b()
        .onTrue(new HandCommands().grabCube(robot.hand));

        specialOpsController.rightBumper()
        .onTrue(new HandCommands().release(robot.hand));


        // Align to April Tag
        specialOpsController.a()
            .onTrue(new AlignToAprilTagCommand(robot.swerveDrive, robot.vision));
  
    }
    
}
