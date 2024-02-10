package frc.robot.interfaces;

import edu.wpi.first.wpilibj2.command.Command;

public interface RobotContainer {
    /**
     * Each frame, update SmartDashboard with current sensor and motor values
     */
    default void robotPeriodic() {}

    Command getAutonomousCommand();
}
