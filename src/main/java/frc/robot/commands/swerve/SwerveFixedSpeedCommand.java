package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * Re-implementation of Kyle & Christopher's autonomous driving routine showing
 * how we can chain commands together to make interesting behaviors.
 */
public class SwerveFixedSpeedCommand extends CommandBase {
    
    private final SwerveDriveSubsystem swerveDrive;
    private final ChassisSpeeds speeds;
    private final double duration;
    private double startTime;
    private double vx;
    private double vy;
    private double vomega;
    private boolean done;

    // this command takes a single set of chassis speeds, and a duration. it
    // will drive the chassis at that speed for that many seconds, and then
    // stop it.
    public SwerveFixedSpeedCommand(SwerveDriveSubsystem swerveDrive, ChassisSpeeds speeds, double duration) {
        this.swerveDrive = swerveDrive;
        this.speeds = speeds;
        this.duration = duration;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        done = false;
    }

    // this command is a lot simpler than the other one, since it only
    // deals with one speed and one time interval.
    @Override
    public void execute() {
        double timeElapsed = Timer.getFPGATimestamp() - startTime;
        if (timeElapsed < duration) {
            swerveDrive.drive(speeds);
        } else {
            swerveDrive.stop();
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    // hark - what foul wizardry is this??!?!!!!?
    // 
    // if your commands are encapsulated nicely, you can combine them together
    // into more complex programs of actions that take place one after another
    // (or all at once, if you're getting fancy).
    //
    // this method creates a command that does the same thing as the KyleAndChristopherCommand,
    // but it's slightly easier to add or remove steps in the program.
    public static Command buildMultiStepProgram(SwerveDriveSubsystem swerveDrive) {

        // a sequential group is a command that runs other commands, one after
        // another in a sequence (hence the name)
        SequentialCommandGroup group = new SequentialCommandGroup();

        // start by waiting for one second
        group.addCommands(new WaitCommand(1));

        // next, drive forward for 3 seconds
        group.addCommands(new SwerveFixedSpeedCommand(
                swerveDrive, 
                new ChassisSpeeds(Units.feetToMeters(1), 0, 0), 
                3));
        
        // next, rotate in place for 3 seconds
        group.addCommands(new SwerveFixedSpeedCommand(swerveDrive,
                new ChassisSpeeds(0, 0, Units.degreesToRadians(90)), 
                3));

        return group;
    }
}
