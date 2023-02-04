
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HandSubsystem;

import frc.robot.commands.ArmTeleopLoopCommand;

/**
 * This is the test bench
 */
public class TestBench extends TimedRobot {

    public static final int CONTROLLER_PORT = 0;

    public HandSubsystem hand;
    public ArmSubsystem arm;

    @Override
    public void robotInit() {

        XboxController controller = new XboxController(CONTROLLER_PORT);

        hand = new HandSubsystem();
        arm = new ArmSubsystem();
        arm.setDefaultCommand(new ArmTeleopLoopCommand(arm, controller));

        TestBenchControlMapping.mapControls(this, controller);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

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
