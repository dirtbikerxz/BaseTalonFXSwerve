package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

/**

 * This is a "skeleton" of a command that does nothing. To make a new command,
 * you can create a copy of this. Don't forget:
 *   - change "ExampleCommand" to something else (e.g. "MyAwesomeCommand")
 *   - make sure the file name corresponds to that (e.g. "My
 * AwesomeCommand.java")
 */

public class AlignToWallCommand extends CommandBase {
    private double currentAngle;
    private final Robot robot;
    private double angleInitial;
    private boolean done;

    public AlignToWallCommand(Robot robot) {

        this.robot = robot;

        // If you are going to use a subsystem in any of your methods, you
        // HAVE to uncomment the corresponding line below.
        addRequirements(robot.swerveDrive);
        // addRequirements(robot.hand);
        // addRequirements(robot.arm);
        // addRequirements(robot.vision);
    }

    @Override
    public void initialize() {
        double northAngle = 0;
        double eastAngle = -90;
        double southAngle = 180;
        double westAngle = 90;
        //going to the west Angle
        
        angleInitial = robot.swerveDrive.getYaw().getDegrees();
        // REPLACE ME with real logic that actually does something
        done = false;
    }

    @Override
    public void execute(){
        // currentAngle = robot.swerveDrive.getYaw().getDegrees();
        // SmartDashboard.putNumber("getYaw", currentAngle);
        // if (currentAngle < 91 && currentAngle > 89){
        //    robot.swerveDrive.drive(0, 0, 0, true);
        //    done = true;
        // } else if (currentAngle > -90 && currentAngle < 90){
        //    robot.swerveDrive.drive(0, 0 , 0.5, true);
        // } else {
        //    robot.swerveDrive.drive(0, 0, -0.5, true );
        // }

        // REPLACE ME with real logic that actually does something
    }

    @Override
    public boolean isFinished() {
        // REPLACE ME with real logic that returns true only when your command is finished running
        return done;
    }
}
