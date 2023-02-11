package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AlignToWallCommand extends CommandBase {
    private double currentDirection;
    private final Robot robot;
    private double directionInitial;
    private boolean done;
    private double wantedDirection;
    private double northDirection = 0;
    private double eastDirection = -90;
    private double southDirection = 180;
    private double westDirection = 90;
    private double toleration = 0.5;
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
    }

    @Override
    public void execute(){
        currentDirection = robot.swerveDrive.getYaw().getDegrees();
        // SmartDashboard.putNumber("getYaw", currentAngle);

        if (currentDirection < toleration + wantedDirection && currentDirection > wantedDirection - toleration){
            
            robot.swerveDrive.drive(0, 0, 0);
            done = true;

        } else if (directionDisposition < -180 || directionDisposition < 180){

            robot.swerveDrive.drive(0, 0, 0.5);

        } else {

            robot.swerveDrive.drive(0, 0, -0.5);
        }
   
        // REPLACE ME with real logic that actually does something
    }

    @Override
    public boolean isFinished() {
        // REPLACE ME with real logic that returns true only when your command is finished running
        return done;
    }
}
