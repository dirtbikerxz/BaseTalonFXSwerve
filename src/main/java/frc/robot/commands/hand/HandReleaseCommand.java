package frc.robot.commands.hand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;
public class HandReleaseCommand extends CommandBase {

    private final HandSubsystem handSubsystem;
    private double seconds;
    private boolean done;

    public HandReleaseCommand(HandSubsystem handSubsystem) {
        this.handSubsystem = handSubsystem;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {
        seconds = 0;
        done = false;
    }

    @Override
    public void execute() {

        if (seconds == 0) {
            handSubsystem.ungrab();
            seconds = Timer.getFPGATimestamp();
        }

        if (Timer.getFPGATimestamp() - seconds > 1.0) {
            handSubsystem.turnOff();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
