package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.other.Ultrasonic;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.commands.moveNewMotor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.NewMotor;

public class ultrasonicAuto extends SequentialCommandGroup {
    Ultrasonic ultrasonic;
    NewMotor s_NewMotor;

    public ultrasonicAuto(Swerve s_Swerve, Ultrasonic ultrasonic){
        s_NewMotor = new NewMotor();
        System.out.println("Ultrasonic Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory firstHalfTrajectory =
            TrajectoryGenerator.generateTrajectory(

                List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0))),
                config);
        Trajectory secondHalfTrajectory =
            TrajectoryGenerator.generateTrajectory(

                List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0))),
                config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand firstHalfTraject = new SwerveControllerCommand(firstHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        SwerveControllerCommand secondHalfTraject = new SwerveControllerCommand(secondHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        PrintCommand ultrasonicValue = new PrintCommand("Ultrasonic: " + ultrasonic.getDistanceValue() + "cm");
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(3);
        ZeroMotorsWaitCommand secondWait = new ZeroMotorsWaitCommand(.5);
        ParallelRaceGroup secondTrajectNewMotor = new ParallelRaceGroup(new moveNewMotor(new NewMotor()), secondHalfTraject);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstHalfTrajectory.getInitialPose())),
            firstHalfTraject,
            ultrasonicValue,
            secondTrajectNewMotor,
            firstWait,
            secondWait
        );
    }
}
