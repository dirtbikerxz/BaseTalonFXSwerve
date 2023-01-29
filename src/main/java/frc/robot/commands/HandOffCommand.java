package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
public class HandOffCommand extends CommandBase {

    private final HandSubsystem handSubsystem;

    public HandOffCommand(HandSubsystem handSubsystem) {
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
