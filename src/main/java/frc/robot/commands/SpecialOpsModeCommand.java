package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;

public class SpecialOpsModeCommand extends CommandBase{

    private final SwerveDriveSubsystem swerveDrive;
    private final XboxController driverController;
    private boolean done;


    private double applyMaxSpeed(double speedConfigure, double maxSpeed){

        if (speedConfigure <= maxSpeed){
            return speedConfigure;
        } else {
            return maxSpeed;
        }

    }

    public SpecialOpsModeCommand(SwerveDriveSubsystem swerveDrive, XboxController driverController) {
        this.swerveDrive = swerveDrive;
        this.driverController = driverController;

        // all commands have to declare which subsystems they need to use;
        // this one only uses the swerve drive, but we could have more
        // complicated commands that use multiple subsystems.
        addRequirements(swerveDrive);
    }

    public void initialize(){
        done = false;
    }

    public void execute(){

        double vx = -driverController.getRawAxis(kLeftX.value); 
        double vy = -driverController.getRawAxis(kLeftY.value); 
        double vomega = -driverController.getRawAxis(kLeftX.value);

        if (driverController.getBackButtonPressed()){
            done = true;
        }

        swerveDrive.drive(vx, vy, vomega, false, 0.3);

    }

    public boolean isFinished(){
        return done;
    }


    
}
