package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoot;

public class TeleopShoot extends Command {
    Shoot mShoot;
    public TeleopShoot (Shoot s_Shoot) {
        this.mShoot = s_Shoot;
    }
    public void execute () {
        mShoot.shooterOn();
    }
    
}
