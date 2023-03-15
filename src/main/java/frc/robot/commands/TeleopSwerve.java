package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

        if (rotateToScoreSup.getAsBoolean()) {
            rotationVal = PID.calculate(s_Swerve.getYaw().getDegrees(), Constants.ROTATE_TO_SCORE_TARGET_ANGLE);
            // rotationVal = MathUtil.inputModulus(rotationVal, -180.0, 180);
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