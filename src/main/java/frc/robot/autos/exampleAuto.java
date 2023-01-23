package frc.robot.autos;

import frc.robot.subsystems.SwerveConfig;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public exampleAuto(SwerveDriveSubsystem s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    kMaxSpeedMetersPerSecond,
                    kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(SwerveConfig.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                kPThetaController, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = null;
            // new SwerveControllerCommand(
            //     exampleTrajectory,
            //     s_Swerve::getPose,
            //     Constants.Swerve.swerveKinematics,
            //     new PIDController(kPXController, 0, 0),
            //     new PIDController(kPYController, 0, 0),
            //     thetaController,
            //     s_Swerve::setModuleStates,
            //     s_Swerve);


        addCommands(
            // new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}