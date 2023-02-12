package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotatingWheelsToADegreeCommand extends CommandBase{

    private double wheelRotatingDirection;
    private boolean done;
    private final SwerveDriveSubsystem swerveDrive;
    private SwerveModuleState [] rotaingModules = new SwerveModuleState [4];
    private double startTime;
    // private double intendedTime;

    public RotatingWheelsToADegreeCommand(Robot robot, double intendedDirection){

        this.wheelRotatingDirection = intendedDirection;
        this.swerveDrive = robot.swerveDrive;

        addRequirements(robot.swerveDrive);

    }

    public void initialize(){

        System.err.println("initialized");

        done = false;

        for (int i = 0; i < 4; i++){
            this.rotaingModules [i] = new SwerveModuleState(0.1,Rotation2d.fromDegrees(wheelRotatingDirection));
        }
    }

    public void execute(){

        swerveDrive.setModuleStates(rotaingModules);

    }

    public boolean isFinished(){

        return true;
        
    }


}