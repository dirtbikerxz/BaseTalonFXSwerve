package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    public Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier rotateToScoreSup;
    private BooleanSupplier rotateToLoadSup;
    private BooleanSupplier slowModeSup;
    private double rotationSpeed;
    private ProfiledPIDController PID;
    private boolean rotating = false;
    // private Timer timer;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier rotateToScoreSup, BooleanSupplier rotateToLoadSup,
            BooleanSupplier slowModeSup, double rotationSpeed) {
        Timer.delay(1.0);
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.rotateToScoreSup = rotateToScoreSup;
        this.rotateToLoadSup = rotateToLoadSup;
        this.slowModeSup = slowModeSup;
        this.rotationSpeed = rotationSpeed;
        
        PID = new ProfiledPIDController(
            Constants.ROTATE_TO_SCORE_KP, 
            Constants.ROTATE_TO_SCORE_KI, 
            Constants.ROTATE_TO_SCORE_KD, 
            new Constraints(Constants.ROTATE_TO_SCORE_VELOCITY, Constants.ROTATE_TO_SCORE_ACCELERATION)
        ); 

        // timer = new Timer();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * rotationSpeed;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /*slowmode*/
        if (slowModeSup.getAsBoolean()) {
            translationVal = translationVal * Constants.SLOW_MODE_PERCENT_TRANSLATION;
            strafeVal = strafeVal * Constants.SLOW_MODE_PERCENT_STRAFE;
            rotationVal = rotationVal * Constants.SLOW_MODE_PERCENT_ROTATION;
        }

        /* Rotate to Score */
        double robot_angle = s_Swerve.getYaw().getDegrees();
        robot_angle = MathUtil.inputModulus(robot_angle, 0, 360);
        double target_angle = 0;

        // if (rotateToScoreSup.getAsBoolean() || rotating) {
        if (robot_angle > 0) {
            target_angle = Constants.ROTATE_TO_SCORE_TARGET_ANGLE;
        } else {
            target_angle = -1 * Constants.ROTATE_TO_SCORE_TARGET_ANGLE;
        }
        // } else if (rotateToLoadSup.getAsBoolean() || rotating) {
        //     target_angle = Constants.ROTATE_TO_LOAD_TARGET_ANGLE;
        // }
        // SmartDashboard.putNumber("rotateToScoreVal", rotateToScoreVal);
        // SmartDashboard.putNumber("robot_angle", robot_angle);
        
        double rotateTo = PID.calculate(robot_angle, target_angle);
        
        if (rotateToScoreSup.getAsBoolean() || /*rotateToLoadSup.getAsBoolean() ||*/ rotating) {
            // timer.start();

            rotating = true;

            rotationVal = rotateTo;

            rotationVal += MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * 0.10;

            SmartDashboard.putNumber("Robot Angle", robot_angle);
            SmartDashboard.putNumber("Target Angle", target_angle);
            SmartDashboard.putNumber("Rotation Value", rotationVal);

            double error = Math.abs(robot_angle - target_angle);

            // SmartDashboard.putNumber("Current Time", timer.get());
            // SmartDashboard.putBoolean("rotating", rotating);

            // TODO: Fix this
            // Currently, it will stop the robot from rotating after 2.0 seconds 
            // but from that point on, you are forced to hold it down.
            // This bug persists between enable/disable cycles and is only solved by restarting robot code.
            // The intended behavior is for it to start a new timer when the button is pushed
            // and stop it when it reaches it's destination or after two seconds.
            // Then the timer should restart the next time that the button is pushed.
            // NOTE: I have taken out the timer implementation
            if (error <= 2.5 && error >= -2.5)  {
                // timer.stop();
                rotating = false;
            } 
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false
        );
    }
}