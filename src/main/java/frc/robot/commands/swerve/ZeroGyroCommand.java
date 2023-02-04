package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Example of a super-simple command that just runs a method on a subsystem.
 */
public class ZeroGyroCommand extends RunCommand {
    
    public ZeroGyroCommand(SwerveDriveSubsystem swerveDrive) {
        super(swerveDrive::zeroGyro, swerveDrive);
    }
    @Override
    public boolean isFinished() {
        return true;
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
   
}
