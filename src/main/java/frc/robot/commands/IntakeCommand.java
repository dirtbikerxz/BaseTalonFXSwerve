package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_subsytem;

        // @param subsystem
    public IntakeCommand(Intake subsystem) {
        m_subsytem = subsystem;

        addRequirements(subsystem);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsytem.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsytem.stopIntake();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
