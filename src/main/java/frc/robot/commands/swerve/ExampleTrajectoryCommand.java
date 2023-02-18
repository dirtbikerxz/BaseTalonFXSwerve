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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.List;

/**
 * Sample command that follows a specified trajectory across the field
 */
public class ExampleTrajectoryCommand extends SequentialCommandGroup {

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

    /** Create the configuration for the trajector */
    private TrajectoryConfig makeTrajectoryConfig() {
        return new TrajectoryConfig(
                MAX_SPEED_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(SwerveConfig.defaultKinematics);
    }

    /**
     * Create the trajectory:
     *   - start at the origin facing the +x direction
     *   - pass through two interior waypoints, making an 's' curve
     *   - end 3 meters ahead of where we started, facing forward
     */
    private Trajectory makeTrajectory(TrajectoryConfig config) {

        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));

        List<Translation2d> waypoints = List.of(
               new Translation2d(1, 0.7),
               new Translation2d(2, -1));

        Pose2d finalPose = new Pose2d(3, 0, new Rotation2d(0));

        return TrajectoryGenerator.generateTrajectory(
                startingPose,
                waypoints,
                finalPose,
                config);
    }

    /**
     * Constructor for the trajectory command
     */
    public ExampleTrajectoryCommand(SwerveDriveSubsystem swerve) {

        SmartDashboard.putNumber("Foo", 2);

        TrajectoryConfig config = makeTrajectoryConfig();

        SmartDashboard.putNumber("Foo", 3);

        Trajectory exampleTrajectory = makeTrajectory(config);

        SmartDashboard.putNumber("Foo", 4);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(PTHETA_CONTROLLER, 0, 0, THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
             new SwerveControllerCommand(
                 exampleTrajectory,
                 swerve::getPose,
                 SwerveConfig.defaultKinematics,
                 new PIDController(PX_CONTROLLER, 0, 0),
                 new PIDController(PY_CONTROLLER, 0, 0),
                 thetaController,
                 swerve::setModuleStates,
                 swerve);

        addCommands(swerveControllerCommand);
    }
}