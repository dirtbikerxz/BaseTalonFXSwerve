package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewMotor;

public class moveNewMotor extends CommandBase{
    NewMotor s_NewMotor;
    public moveNewMotor(NewMotor subsystem){
        s_NewMotor = subsystem;
        addRequirements(s_NewMotor);
    }

    @Override
    public void execute() {
        // System.out.println("Running!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        s_NewMotor.move();
    }

    @Override
    public void end(boolean interrupted) {
        s_NewMotor.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
