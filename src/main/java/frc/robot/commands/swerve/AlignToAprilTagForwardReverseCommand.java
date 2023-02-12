package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTag;

public class AlignToAprilTagForwardReverseCommand extends CommandBase {

    public static final double TARGET_VALUE = 1.0; // TODO determine the correct value
    public static final double TOLERANCE = 0.1; // TODO determine the currect value
    public static final double SPEED = Units.metersToFeet(1.0); // TODO determine the correct value

    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem vision;
    private boolean done;

    public AlignToAprilTagForwardReverseCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem vision) {

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

        // larger values mean we're further away from the tag
        
        double distance = tag.getForwardReverseDistance();

        // close enough; stop driving
        if (Math.abs(distance - TARGET_VALUE) < TOLERANCE) {
            swerveDrive.stop();
            done = true;
        } 
        
        // too close; back up
        else if (distance < TARGET_VALUE) {
            ChassisSpeeds speeds = new ChassisSpeeds(-SPEED, 0, 0);
            swerveDrive.drive(speeds);
            done = false;
        } 
        
        // too far away, move forward
        else {
            ChassisSpeeds speeds = new ChassisSpeeds(SPEED, 0, 0);
            swerveDrive.drive(speeds);
            done = false;
        }
    }

    public boolean isFinished() {
        return done;
    }
}
