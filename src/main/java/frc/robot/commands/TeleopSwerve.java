package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    public Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier rotateToScoreSup;
    private double rotationSpeed;
    private ProfiledPIDController PID;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier rotateToScoreSup, 
            double rotationSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.rotateToScoreSup = rotateToScoreSup;
        this.rotationSpeed = rotationSpeed;
        
        PID = new ProfiledPIDController(
            Constants.ROTATE_TO_SCORE_KP, 
            Constants.ROTATE_TO_SCORE_KI, 
            Constants.ROTATE_TO_SCORE_KD, 
            new Constraints(Constants.ROTATE_TO_SCORE_ACCELERATION, Constants.ROTATE_TO_SCORE_VELOCITY)
        ); 
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * rotationSpeed;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double robot_angle = s_Swerve.getYaw().getDegrees();
        robot_angle = MathUtil.inputModulus(robot_angle, 0, 360);
        double rotateToScoreVal;
        if (robot_angle > 0) {
            rotateToScoreVal = PID.calculate(robot_angle, Constants.ROTATE_TO_SCORE_TARGET_ANGLE);
        } else {
            rotateToScoreVal = PID.calculate(robot_angle, -1 * Constants.ROTATE_TO_SCORE_TARGET_ANGLE);
        }
        SmartDashboard.putNumber("rotateToScoreVal", rotateToScoreVal);
        SmartDashboard.putNumber("robot_angle", robot_angle);

        if (rotateToScoreSup.getAsBoolean()) {
            rotationVal = rotateToScoreVal;
        }
        SmartDashboard.putNumber("Angle", s_Swerve.getYaw().getDegrees());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}