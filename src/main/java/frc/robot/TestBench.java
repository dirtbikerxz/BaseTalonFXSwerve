
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;

/**
 * This is the test bench
 */
public class TestBench extends TimedRobot {

    public static final int CONTROLLER_PORT = 0;

    public HandSubsystem hand;
    public ArmSubsystem arm;

    @Override
    public void robotInit() {

        hand = new HandSubsystem();

        TestBenchControlMapping.mapControls(this, new CommandXboxController(CONTROLLER_PORT));
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
