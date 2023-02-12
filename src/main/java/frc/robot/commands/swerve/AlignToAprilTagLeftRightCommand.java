package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTag;

public class AlignToAprilTagLeftRightCommand extends CommandBase {

    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem vision;
    private boolean done;

    public AlignToAprilTagLeftRightCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem vision) {

        this.swerveDrive = swerveDrive;
        this.vision = vision;

        addRequirements(swerveDrive);
        // we don't need to require vision, because we're not commanding it to do anything
    }

    public void initialize() {
        done = false;
    }

    public void execute() {

        AprilTag tag = vision.getAprilTag();

        // If there is no tag in view, we'll just stop and be done.
        // TODO is it possible this is a transient error, and we should keep trying?
        if (tag == null) {
            swerveDrive.stop();
            done = true;
            return;
        }

        // larger values means the tag is further to the RIGHT in our view
        double distance = tag.getLeftRightDistance();

        // TODO use tag info to drive into correct position
    }

    public boolean isFinished() {
        return done;
    }
}
