package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NewMotor;

public class moveNewMotor extends CommandBase{
    NewMotor s_NewMotor;
    public moveNewMotor(NewMotor subsystem){
        this.s_NewMotor = subsystem;
    }

    @Override
    public void initialize() {
        addRequirements(s_NewMotor);
    }

    @Override
    public void execute() {
        s_NewMotor.move();
    }

    
    @Override
    public void end(boolean interrupted) {
        s_NewMotor.stop();
    }

}
