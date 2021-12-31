package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Ultrasonic;

public class ultrasonicAuto extends SequentialCommandGroup {
    private Ultrasonic ultrasonic = new Ultrasonic();
    public ultrasonicAuto(Swerve s_Swerve){
        System.out.println("Ultrasonic Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory firstHalfTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(5, 5, new Rotation2d(90)),
                config);

        Trajectory secondHalfTrajectory =
            TrajectoryGenerator.generateTrajectory( List.of(new Pose2d(2, 3, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0))), config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand firstHalfTraject = new SwerveControllerCommand(firstHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        PrintCommand sensorVal = new PrintCommand(ultrasonic.getDistanceValue() + " cm");
        SwerveControllerCommand secondHalfTraject = new SwerveControllerCommand(secondHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstHalfTrajectory.getInitialPose())),
            firstHalfTraject, 
            sensorVal, 
            secondHalfTraject
        );
    }
}