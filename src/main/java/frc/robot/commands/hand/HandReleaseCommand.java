package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
        handSubsystem._Charge.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public boolean isFinished() {
        // TODO what do we do here?
        return true;
    }
}
