package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.autos.exampleAuto;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;


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
        retentionCounter = 15;
        // Tuening : increase the retention counter

    }

    public void execute(){

        nowPitch = Math.abs(swerveDrive.getPitch());

        if(inputController.getBackButtonPressed()){

            done = true;

        }

        //System.err.println(nowPitch - thenPitch);

        if (nowPitch - thenPitch < -0.45){

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