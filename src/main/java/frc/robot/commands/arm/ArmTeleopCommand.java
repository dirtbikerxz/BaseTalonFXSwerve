package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmTeleopCommand extends CommandBase {

    private final ArmSubsystem arm;
    private final DoubleSupplier rotateSupplier;
    private final DoubleSupplier extendSupplier;

    public ArmTeleopCommand(ArmSubsystem arm, DoubleSupplier rotateSupplier, DoubleSupplier extendSupplier) {
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
