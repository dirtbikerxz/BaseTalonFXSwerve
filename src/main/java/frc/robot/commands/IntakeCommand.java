package frc.robot.commands;

import frc.robot.Constants.Swerve.IntakeState;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Intake m_subsytem;
    private final DigitalInput input;
    private IntakeState intakePosition;

        // @param subsystem
    public IntakeCommand(Intake subsystem) {
        m_subsytem = subsystem;
        input = new DigitalInput(1);
        intakePosition = IntakeState.Deactivated;

        addRequirements(subsystem);
    }


    @Override
    public void initialize() {}

    @Override
    public void execute() {

        // m_subsytem.runIntake();
        if (!input.get() && intakePosition == IntakeState.Deactivated)
        {
            m_subsytem.runIntake();
            intakePosition = IntakeState.Activated;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (input.get() && intakePosition == IntakeState.Activated)
        {
            m_subsytem.reverseIntake();
            intakePosition = IntakeState.Deactivated;
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
