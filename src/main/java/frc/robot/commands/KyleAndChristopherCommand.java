package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Re-implementation of Kyle & Christopher's autonomous driving routine using
 * the command framework. For an overview of commands, read this:
 *    https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
 */

// like all other commands, this one "extends CommandBase"
public class KyleAndChristopherCommand extends CommandBase {
    
    private final SwerveDriveSubsystem swerveDrive;
    private double startTime;
    private boolean done;

    public KyleAndChristopherCommand(SwerveDriveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;

        // all commands have to declare which subsystems they need to use;
        // this one only uses the swerve drive, but we could have more
        // complicated commands that use multiple subsystems.
        addRequirements(swerveDrive);
    }

    // this gets called once when your command is ready to run.
    // use it to initialize anything that needs to be reset. in
    // this case, we'll capture the start time.
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        done = false;
    }

    // this gets called every 20ms while your command is running;
    // it's just like one of the xxxPeriodic methods.
    @Override
    public void execute() {

        double timeElapsed = Timer.getFPGATimestamp() - startTime;

        // for the first second, we'll stay still.
        if (timeElapsed < 1) {
            swerveDrive.drive(0, 0, 0, true);
        }

        // for the next three seconds, we'll drive forward.
        else if (timeElapsed < 4) {
            swerveDrive.drive(Units.feetToMeters(1), 0, 0, true);  
        }

        // for the next three seconds, we'll turn in place.
        else if (timeElapsed < 7) {
            swerveDrive.drive(0, 0, Units.degreesToRadians(30), true);
        }

        // otherwise, we're done.
        else {
            done = true;
        }
    }

    // this gets called right after execute(), to see if the
    // command needs to keep running. as soon as it returns
    // true, this command is over and execute() will stop
    // getting called.
    @Override
    public boolean isFinished() {
        return done;
    }

    // only one command can be using a subsystem at any given
    // time. if a new command comes in that wants to use a
    // subsystem that's currently in use, the scheduler may
    // "interrupt" the existing command.
    // 
    // EXAMPLE - we are running a command that positions the
    // robot in front of the grid, and the driver changes their
    // mind and wants to drive the robot to another spot
    // manually.
    // 
    // this function lets us tell the system what to do in
    // the case of an interruption.
    @Override
    public InterruptionBehavior getInterruptionBehavior() {

        // this is the default behavior - we are saying "if
        // I'm interrupted, that's OK, I can be canceled".
        // return InterruptionBehavior.kCancelSelf;

        // in this case, we want our entire routing to run
        // until it's finished, so we won't allow ourselves
        // to be canceled - instead, the new command will
        // get canceled.
        return InterruptionBehavior.kCancelIncoming;
    }
}
