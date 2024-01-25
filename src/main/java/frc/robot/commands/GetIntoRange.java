package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class GetIntoRange extends Command{
    private Vision vision;
    private Swerve swerve;
    private Supplier<Pose2d> poseProvider;
    private PhotonCamera camera;

    private static TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 1);
    private static TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 1); 
    private static TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(1, 1); 
    
    private static final Transform3d tagToGoal = new Transform3d(new Translation3d(1.5,0.0,0), new Rotation3d(0.0, 0.0, Math.PI));

    private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, xConstraints);
    private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, yConstraints);
    private ProfiledPIDController rotController = new ProfiledPIDController(0, 0, 0, rotConstraints);

    public GetIntoRange(Vision vision, Swerve swerve, Supplier<Pose2d> poseProvider, PhotonCamera camera) {
        this.vision = vision;
        this.swerve = swerve;
        this.poseProvider = poseProvider;
        this.camera = camera;


        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        rotController.setTolerance(Units.degreesToRadians(2));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        var robotPose = poseProvider.get();
        rotController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());    
    }


    @Override
    public void execute() {
        var robotPose2d = poseProvider.get();
        var robotPose = new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians())
            );

        var photonRes = camera.getLatestResult();

        if (photonRes.hasTargets()) {
            var cameraPose = robotPose.transformBy(tagToGoal);
            var camToTarget = photonRes.getBestTarget().getBestCameraToTarget();
            var targetPose = cameraPose.transformBy(camToTarget);

            var goalPose = targetPose.transformBy(camToTarget);

            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            rotController.setGoal(goalPose.getRotation().getAngle());
           
        }

        var xSpeed = xController.calculate(robotPose.getX());

        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());

        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var rotSpeed = rotController.calculate(robotPose.getRotation().getAngle());

        if (rotController.atGoal()) {
            rotSpeed = 0;
        }

        swerve.drive(
            new Translation2d(xSpeed, ySpeed), robotPose2d.getRotation().getRadians(), false, true);
    }

        @Override
        public void end(boolean interrupted) {
        
        }
}
