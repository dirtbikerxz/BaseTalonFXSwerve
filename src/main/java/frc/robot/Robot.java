package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonomousCommand;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This is where all the parts of our robot are created.
 * 
 * DON'T add new control logic to this class. Instead, do the following:
 * 
 *  - Create a new command that will wrap up your logic. See ExampleCommand.java
 *  in the commands folder, or look at some of the other commands in there to
 *  see how they work.
 * 
 *  - Update RobotControlMapping.java to establish a button or something that
 *  will trigger your command.
 */
public class Robot extends TimedRobot {

    public static final int DRIVE_PORT = 0;

    public SwerveDriveSubsystem swerveDrive;
    public HandSubsystem hand;
    public ArmSubsystem arm;
    public VisionSubsystem vision;
    public AutonomousCommand autonomousCommand;

    @Override
    public void robotInit() {

        // create the swerve drive and establish the default control mapping
        // for driving in teleop mode
        swerveDrive = new SwerveDriveSubsystem();
        vision = new VisionSubsystem(true);

        // do any additional control mapping that needs to be done
        RobotControlMapping.mapDriverControls(this, new CommandXboxController(DRIVE_PORT));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() { }

    @Override
    public void disabledPeriodic() { }

    @Override
    public void autonomousInit() {
        autonomousCommand = new AutonomousCommand(this);
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        swerveDrive.zeroGyro();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();    
    }

    @Override
    public void testPeriodic() {    
    }
}
