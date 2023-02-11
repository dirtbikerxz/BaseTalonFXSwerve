package frc.robot.commands.hand;

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

    }

    @Override
    public void execute() {
        handSubsystem.release();

    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
