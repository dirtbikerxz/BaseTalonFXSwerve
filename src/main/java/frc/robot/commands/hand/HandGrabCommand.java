package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HandSubsystem;


public class HandGrabCommand extends CommandBase {

    public static final int CUBE = 0;
    public static final int CONE = 1;

    final HandSubsystem handSubsystem;
    private final int which;



    public HandGrabCommand(HandSubsystem handSubsystem, int which) {
        this.handSubsystem = handSubsystem;
        this.which = which;
        addRequirements(handSubsystem);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        
        if (which == CONE) {
            handSubsystem.grabCone();
           
        }
        if (which == CUBE) {
            handSubsystem.grabCube();

        }
    }

    @Override
    public boolean isFinished() {
     return true;
    }
}
