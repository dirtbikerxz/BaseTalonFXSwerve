package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmCalibrationCommand extends CommandBase {

    public static final double ROTATE_SPEED = 0.2;
    public static final double EXTEND_SPEED = 0.2;

    private final ArmSubsystem arm;
    private boolean done;

    public ArmCalibrationCommand(ArmSubsystem arm) {
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
        done = arm.calibrate(ROTATE_SPEED, EXTEND_SPEED);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
