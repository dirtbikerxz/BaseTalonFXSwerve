package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.Arrays;
import java.util.List;

/**
 * Sample command that follows a specified trajectory across the field
 */
public class RelativeTrajectoryCommand extends SequentialCommandGroup {

    public static final double MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double PX_CONTROLLER = 1;
    public static final double PY_CONTROLLER = 1;
    public static final double PTHETA_CONTROLLER = 1;

    /* Constraint for the motion profiled robot angle controller */
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    /* Create the configuration for a trajectory */
    public static TrajectoryConfig makeTrajectoryConfig() {
        return new TrajectoryConfig(
                MAX_SPEED_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(SwerveConfig.defaultKinematics);
    }

    public static ProfiledPIDController makeThetaController() {
        ProfiledPIDController c = new ProfiledPIDController(PTHETA_CONTROLLER, 0, 0, THETA_CONTROLLER_CONSTRAINTS);
        c.enableContinuousInput(-Math.PI, Math.PI);
        return c;
    }

    /* Makes a Trajectory the keeps the robot facing the same way through the supplied points */
    public static Trajectory makeTrajectory(Translation2d... points) {
        TrajectoryConfig c = makeTrajectoryConfig();
        Pose2d s = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d e = new Pose2d(points[points.length], new Rotation2d(0));
        List<Translation2d> w = Arrays.asList(points).subList(0, points.length-1);
        return TrajectoryGenerator.generateTrajectory(s, w, e, c);
    }

    /**
     * Makes a command starting at the current position and moving through
     * the specified points (relative to current position/heading)
     */
    public static Command makeCommand(SwerveDriveSubsystem drive, Translation2d... points) {
        return new SwerveControllerCommand(
            makeTrajectory(points),
            drive::getPose,
            SwerveConfig.defaultKinematics,
            new PIDController(PX_CONTROLLER, 0, 0),
            new PIDController(PY_CONTROLLER, 0, 0),
            makeThetaController(),
            drive::setModuleStates,
            drive);
    }
}