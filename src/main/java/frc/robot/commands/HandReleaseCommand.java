package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;

public class HandReleaseCommand extends CommandBase {

    private final HandSubsystem handSubsystem;

    public HandReleaseCommand(HandSubsystem handSubsystem) {
        this.handSubsystem = handSubsystem;
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
