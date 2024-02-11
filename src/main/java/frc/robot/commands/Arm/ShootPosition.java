package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;

public class ShootPosition extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    Arm arm;

    public ShootPosition(Arm subsystem) {
        arm = subsystem;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setShoot();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopSet();
    }

    @Override
    public boolean isFinished() {
        return arm.endConditionShoot();
    }

}
