package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command{
    Shooter shooter;

    public ShootNote(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shootNote();
    }

    @Override
    public void end(boolean interupted) {
        shooter.stop();
    }
}
