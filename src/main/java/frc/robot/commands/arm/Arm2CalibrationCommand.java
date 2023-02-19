package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem2;

public class Arm2CalibrationCommand extends CommandBase {

    public static final double ROTATE_SPEED = 0.2;
    public static final double EXTEND_SPEED = 0.2;

    private final ArmSubsystem2 arm;
    private boolean done;

    public Arm2CalibrationCommand(ArmSubsystem2 arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        done = false;
        arm.clearLimits();
    }

    @Override
    public void execute() {
        done = arm.captureLimits(ROTATE_SPEED, EXTEND_SPEED);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
