package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveTest;

/**
 * Command that assigns a specific speed to the drive motors of the swerve drive
 */
public class TestSwerveDriveMotors extends Command {
    private static final double DEFAULT_SPEED = 0.50;
    private final int mModule;
    private final double mSpeed;
    private final SwerveTest mSwerve;

    public TestSwerveDriveMotors(SwerveTest swerve, int module) {
        this(swerve, module, DEFAULT_SPEED);
    }

    public TestSwerveDriveMotors(SwerveTest swerve, int module, double speed) {
        addRequirements(swerve);
        this.mSwerve = swerve;
        this.mModule = module;
        this.mSpeed = speed;
    }

    public void initialize() {
        mSwerve.setDriveSpeed(mModule, mSpeed);
    }

    public void execute() {
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        mSwerve.setDriveSpeed(mModule, 0.0);
    }
}
