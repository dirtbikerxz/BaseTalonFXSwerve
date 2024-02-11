package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IndexNote extends Command{
    Shooter shooter;

    public IndexNote(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.indexNote();
    }

    @Override
    public void end(boolean interupted) {
        shooter.stop();
    }
}
