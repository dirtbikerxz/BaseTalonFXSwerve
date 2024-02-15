package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveTest;

/**
 * Command that assigns a specific speed to the drive motors of the swerve drive
 */
public class TestSwerveSteerMotors extends Command {
    private static final double DEFAULT_SPEED = 0.50;
    public final int mModule;
    private final double mPosition;
    private final SwerveTest mSwerve;

    public TestSwerveSteerMotors(SwerveTest swerve, int module) {
        this(swerve, module, DEFAULT_SPEED);
    }

    public TestSwerveSteerMotors(SwerveTest swerve, int module, double position) {
        this.mSwerve = swerve;
        this.mModule = module;
        this.mPosition = position;
    }

    public void initialize() {
        mSwerve.setSteerPosition(mModule, mPosition);
    }

    public void execute() {
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        mSwerve.setSteerPosition(mModule, 0.0);
    }
}
