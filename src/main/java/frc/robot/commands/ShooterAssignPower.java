package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterAssignPower extends Command {
    private final Shooter mShooter;

    /**
     * Command that assigns the given power to the Shooter subsystem
     *
     * @param shooter Shooter subsystem to set power for
     * @param power Power [-1.0, 1.0] to assign to shooter motors
     */
    public ShooterAssignPower(Shooter shooter, double power) {
        this.mShooter = shooter;
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() { }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }
}
