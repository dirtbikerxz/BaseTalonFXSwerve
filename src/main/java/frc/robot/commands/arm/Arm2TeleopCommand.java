package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem2;

import java.util.function.DoubleSupplier;

public class Arm2TeleopCommand extends CommandBase {

    private final ArmSubsystem2 arm;
    private final DoubleSupplier rotateSupplier;
    private final DoubleSupplier extendSupplier;

    public Arm2TeleopCommand(ArmSubsystem2 arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {
        this.arm = arm;
        this.rotateSupplier = rotateSupplier;
        this.extendSupplier = extendSupplier;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.moveAt(rotateSupplier.getAsDouble(), extendSupplier.getAsDouble());
    }
}
