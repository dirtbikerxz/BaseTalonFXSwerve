package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class TestSwerve extends Command {
    private Swerve mSwerve;
    private Translation2d mHeading;
    private double mRotation;

    public TestSwerve(Swerve swerve, Translation2d heading, double rotation) {
        addRequirements(swerve);
        this.mSwerve = swerve;
        this.mHeading = heading;
        this.mRotation = rotation;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mSwerve.drive(mHeading, mRotation, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
