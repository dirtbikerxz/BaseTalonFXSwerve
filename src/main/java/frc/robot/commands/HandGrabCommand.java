package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;

public class HandGrabCommand extends CommandBase {

    public static final int CUBE = 0;
    public static final int CONE = 1;

    private final HandSubsystem handSubsystem;
    private final int which;

    public HandGrabCommand(HandSubsystem handSubsystem, int which) {
        this.handSubsystem = handSubsystem;
        this.which = which;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {
        // TODO what do we do here?
    }

    @Override
    public void execute() {
        // TODO what do we do here?
    }

    @Override
    public boolean isFinished() {
        // TODO what do we do here?
        return true;
    }
}
