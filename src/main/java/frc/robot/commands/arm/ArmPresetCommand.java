package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPresetCommand extends CommandBase {

    // TODO calculate preset positions
    public static final double [] TRAVEL_POSITION = { 150.0, -150.0 };
    public static final double [] PICKUP_POSITION = { 2.0, 3.0 };
    public static final double [] HIGH_POSITION = { 2.0, 3.0 };
    public static final double [] MIDDLE_POSITION = { 2.0, 3.0 };
    public static final double [] LOW_POSITION = { 2.0, 3.0 };
    public static final double [] BALANCE_POSITION = { 2.0, 3.0 };
    public static final double [] LOAD_POSITION = { 2.0, 3.0 };

    // TODO tune me with the actual arm
    public static final double ROTATE_FAST = 0.8;
    public static final double ROTATE_SLOW = 0.2;
    public static final double ROTATE_TRANSITION_POINT = 35;
    public static final double ROTATE_TOLERANCE = 5;

    // TODO tune me with the actual arm
    public static final double EXTEND_FAST = 0.8;
    public static final double EXTEND_SLOW = 0.2;
    public static final double EXTEND_TRANSITION_POINT = 35;
    public static final double EXTEND_TOLERANCE = 5;

    private final ArmSubsystem arm;
    private final double targetRotation;
    private final double targetExtension;
    private boolean done;

    public ArmPresetCommand(ArmSubsystem arm, double [] preset) {
        this.arm = arm;
        this.targetRotation = preset[0];
        this.targetExtension = preset[1];
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {

        double rotationError = targetRotation - arm.getRotatorPosition();
        double percentRotate = calculateRotationSpeed(Math.abs(rotationError));
        if (rotationError > 0.0) {
            percentRotate = -percentRotate;
        }

        double extensionError = targetExtension - arm.getExtenderPosition();
        double percentExtend = calculateExtensionSpeed(Math.abs(extensionError));
        if (extensionError > 0.0) {
            percentExtend = -percentExtend;
        }

        arm.moveAt(percentRotate, percentExtend);
        done = (percentRotate == 0.0 && percentExtend == 0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    // TODO replace me with a ratio or a PID controller?
    private double calculateRotationSpeed(double absoluteError) {
        if (absoluteError < ROTATE_TOLERANCE) {
            return 0.0;
        }
        if (absoluteError < ROTATE_TRANSITION_POINT) {
            return ROTATE_SLOW;
        }
        return ROTATE_FAST;
    }

    // TODO replace me with a ratio or a PID controller?
    private double calculateExtensionSpeed(double absoluteError) {
        if (absoluteError < EXTEND_TOLERANCE) {
            return 0.0;
        }
        if (absoluteError < EXTEND_TRANSITION_POINT) {
            return EXTEND_SLOW;
        }
        return EXTEND_FAST;
    }
}
