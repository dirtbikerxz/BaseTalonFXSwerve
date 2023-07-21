package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private BooleanSupplier slowModeSup;
    private double rotationSpeed;
    private ProfiledPIDController PID;
    private DoubleSupplier targetRotation;
    private boolean isAutoRotating;
    private static final double AUTO_ROTATE_DEADBAND = 0.3;
    // private Timer timer;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier targetRotation,
            BooleanSupplier slowModeSup, double rotationSpeed, boolean isAutoRotating) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowModeSup = slowModeSup;
        this.rotationSpeed = rotationSpeed;
        this.targetRotation = targetRotation;
        this.isAutoRotating = isAutoRotating;
        
        PID = new ProfiledPIDController(
            Constants.ROTATE_KP, 
            Constants.ROTATE_KI, 
            Constants.ROTATE_KD, 
            new Constraints(Constants.ROTATE_VELOCITY, Constants.ROTATE_ACCELERATION)
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
        if (-AUTO_ROTATE_DEADBAND <= rotationVal && rotationVal <= AUTO_ROTATE_DEADBAND && isAutoRotating) {
            double yaw = s_Swerve.getYaw().getDegrees() % 360;

            // convert the yaw from [-360, 360] to [-180, 180]
            if (yaw > 180) {
                yaw = yaw - 360;
            } else if (yaw < -180) {
                yaw = yaw + 360;
            }

            // Convert to Rotation2d for rotate command
            Rotation2d rotationYaw = Rotation2d.fromDegrees(yaw);

            // Get the target rotationVal as a Rotation2d object
            Rotation2d targetRotationVal = Rotation2d.fromDegrees(targetRotation.getAsDouble());

            // Rotate rotationYaw to make the 0 of it at the target angle
            // If targetRotationVal is 180 degrees the rotationYaw's 0 will be pointing towards 180
            rotationYaw = rotationYaw.rotateBy(targetRotationVal);

            // double error = yaw - targetRotationVal;
            //SmartDashboard.putNumber("debug/yaw", yaw);
            // SmartDashboard.putNumber("debug/targetRotationVal", targetRotationVal);
            // SmartDashboard.putNumber("debug/error", error);
            // if (Math.abs(error) > 180) {
            //     yaw = Math.abs(360 - yaw);
            // }
            // SmartDashboard.putNumber("debug/calculated yaw", yaw);

            SmartDashboard.putNumber("debug/calculated yaw", rotationYaw.getDegrees());
            rotationVal = PID.calculate(rotationYaw.getDegrees(), 0.0); 
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}