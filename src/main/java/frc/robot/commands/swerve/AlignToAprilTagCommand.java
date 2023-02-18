package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.vision.AprilTag;

public class AlignToAprilTagCommand extends CommandBase {

    public static final double TARGET_Z = 3;
    public static final double TOLERANCE = 0.075;
    public static final double SPEED = 0.1;
    
    private final SwerveDriveSubsystem swerveDrive;
    private final VisionSubsystem vision;
    private boolean done;

    public AlignToAprilTagCommand(SwerveDriveSubsystem swerveDrive, VisionSubsystem vision) {

        this.swerveDrive = swerveDrive;
        this.vision = vision;

        addRequirements(swerveDrive);
        // we don't need to require vision, because we're not commanding it to do anything
    }

    public void initialize() {
        done = false;
    }

    private double determineSpeedX(AprilTag tag) {
        double distance = tag.getForwardReverseDistance();
        if (Math.abs(distance - TARGET_Z) < TOLERANCE) {
            return 0.0;
        } else if (distance < TARGET_Z) {
            return -SPEED;
        } else {
            return SPEED;
        }
    }

    private double determineSpeedY(AprilTag tag) {
        double distance = tag.getLeftRightDistance();
        if (Math.abs(distance) < TOLERANCE) {
            return 0;
        } else if (distance < 0) {
            return -SPEED;
        } else {
            return SPEED;
        }
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

        double vx = determineSpeedX(tag);
        double vy = determineSpeedY(tag);

        if (vx == 0 && vy == 0) {
            System.err.println("close enough; stopping");
            swerveDrive.stop();
            done = true;
            return;
        }

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, 0.0);    
        System.err.println("got tag "+tag+"; applying speed = "+speeds);
        swerveDrive.drive(speeds);
        done = false;
    }

    public boolean isFinished() {
        return done;
    }
}
