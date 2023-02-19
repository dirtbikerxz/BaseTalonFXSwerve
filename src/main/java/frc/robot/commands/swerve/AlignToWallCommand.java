package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignToWallCommand extends CommandBase {
    private double currentDirection;
    private final Robot robot;
    private double directionInitial;
    private boolean done;
    private double wantedDirection;
    private double toleration = 1;
    private double directionDisposition;

    public AlignToWallCommand(Robot robot, double wantedDirection) {

        this.robot = robot;
        this.wantedDirection = wantedDirection;

        // If you are going to use a subsystem in any of your methods, you
        // HAVE to uncomment the corresponding line below.
        addRequirements(robot.swerveDrive);
        // addRequirements(robot.hand);
        // addRequirements(robot.arm);
        // addRequirements(robot.vision);
    }

    @Override
    public void initialize() {

        directionInitial = robot.swerveDrive.getYaw().getDegrees();
        directionDisposition = wantedDirection - directionInitial;
        // REPLACE ME with real logic that actually does something
        done = false;
        System.err.println(directionDisposition);
    }

    @Override
    public void execute(){
        currentDirection = robot.swerveDrive.getYaw().getDegrees();
        //System.err.println("StartingCommand");
        //System.err.println(currentDirection);
        SmartDashboard.putNumber("getYaw", currentDirection);

        if (currentDirection < toleration + wantedDirection && currentDirection > wantedDirection - toleration){
            
            robot.swerveDrive.drive(0, 0, 0);
            done = true;

        } else if (directionDisposition < -180 || (directionDisposition < 180 && directionDisposition > 0)){

            robot.swerveDrive.drive(0, 0, 0.3);

        } else {

            robot.swerveDrive.drive(0, 0, -0.3);
        }
   
        // REPLACE ME with real logic that actually does something
    }

    @Override
    public boolean isFinished() {
        // REPLACE ME with real logic that returns true only when your command is finished running
        return done;
    }
}
