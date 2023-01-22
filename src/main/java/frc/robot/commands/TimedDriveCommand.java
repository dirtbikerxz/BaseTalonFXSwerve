package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class TimedDriveCommand extends CommandBase {

    public static final ChassisSpeeds STOP = new ChassisSpeeds(0, 0, 0);
    
    private final SwerveDrive drive;
    private final ChassisSpeeds speeds;
    private final double duration;
    private double endTime;
    private boolean done;

    public TimedDriveCommand(SwerveDrive drive, ChassisSpeeds speeds, double duration) {
        this.drive = drive;
        this.speeds = speeds;
        this.duration = duration;
        addRequirements(drive);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + duration;
        done = false;
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void execute() {

        ChassisSpeeds nextSpeed = speeds;
        if (Timer.getFPGATimestamp() >= endTime) {
            done = true;
            nextSpeed = STOP;
        }

        drive.drive(nextSpeed);
    }

    public static Builder buildProgram(SwerveDrive drive) {
        return new Builder(drive);
    }

    public static class Builder extends ArrayList<TimedDriveCommand> {

        private SwerveDrive drive;

        public Builder(SwerveDrive drive) {
            this.drive = drive;
        }

        public Builder add(ChassisSpeeds speeds, double seconds) {
            add(new TimedDriveCommand(drive, speeds, seconds));
            return this;
        }

        public Command toCommand() {
            Command [] commands = toArray(l -> new Command[l]);
            return new SequentialCommandGroup(commands);
        }
    }
}
