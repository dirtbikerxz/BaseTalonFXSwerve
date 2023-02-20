package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;

public class RotatingWheelsToADegreeCommand extends CommandBase {

    private double wheelRotatingDirection;
    private final SwerveDriveSubsystem swerveDrive;
    private SwerveModuleState [] rotaingModules = new SwerveModuleState [4];

    public RotatingWheelsToADegreeCommand(Robot robot, double intendedDirection){
        this.wheelRotatingDirection = intendedDirection;
        this.swerveDrive = robot.swerveDrive;
        addRequirements(robot.swerveDrive);
    }

    public void initialize() {
        for (int i=0; i<4; i++) {
            this.rotaingModules[i] = new SwerveModuleState(0.1,Rotation2d.fromDegrees(wheelRotatingDirection));
        }
    }

    public void execute(){
        swerveDrive.setModuleStates(rotaingModules);
    }

    public boolean isFinished(){
        return true;
    }
}