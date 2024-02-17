package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class LockAprilTag extends Command {    
    private Swerve s_Swerve;    
    private Eyes s_Eyes;

    public LockAprilTag(Swerve s_Swerve, Eyes s_Eyes) {
        this.s_Swerve = s_Swerve;
        this.s_Eyes = s_Eyes;
        addRequirements(s_Swerve, s_Eyes);

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        Pose2d robotPose = s_Swerve.m_poseEstimator.getEstimatedPosition();
        Pose3d targetPose = s_Swerve.getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle = Math.atan((targetY - robotY) / (targetX - robotX));

        /* Drive */
        
    }

      // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false; 
  }
}