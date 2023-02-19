package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;


public class ParkingOnThePlatformCommand extends CommandBase{

    private final SwerveDriveSubsystem swerveDrive;
    private double thenPitch;
    private double nowPitch;
    private boolean done;
    private double retentionCounter;
    private final XboxController inputController;
    

    
    public ParkingOnThePlatformCommand(Robot robot, XboxController inputController){

        this.swerveDrive = robot.swerveDrive;
        this.inputController = inputController;
        addRequirements(robot.swerveDrive);

    }

    public void initialize(){

        thenPitch = Math.abs(swerveDrive.getPitch());
        done = false;
        retentionCounter = 10;
        // Tuening : increase the retention counter

    }

    public void execute(){

        nowPitch = Math.abs(swerveDrive.getPitch());

        if(inputController.getBackButtonPressed()){

            done = true;

        }

        //System.err.println(nowPitch - thenPitch);

        if (nowPitch - thenPitch < -0.8){

            //Tuening: decrease the threashold

            swerveDrive.drive(0, 0, 0);
            retentionCounter -= 1;

        } else {

            swerveDrive.drive(0.65, 0, 0);
            thenPitch = Math.abs(swerveDrive.getPitch());
            
        }

        if (retentionCounter == 0){

            done = true;

        }
    }

    public boolean isFinished(){

        return done;

    }

}