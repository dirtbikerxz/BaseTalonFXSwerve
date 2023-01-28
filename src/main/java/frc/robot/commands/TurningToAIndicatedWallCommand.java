package frc.robot.commands;
import java.lang.ModuleLayer.Controller;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotControlMapping;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/**

 * This is a "skeleton" of a command that does nothing. To make a new command,
 * you can create a copy of this. Don't forget:
 *   - change "ExampleCommand" to something else (e.g. "MyAwesomeCommand")
 *   - make sure the file name corresponds to that (e.g. "MyAwesomeCommand.java")
 */

public class TurningToAIndicatedWallCommand extends CommandBase {
    private double currentAngle;
    private final Robot robot;
    private double angleInitial;

    public TurningToAIndicatedWallCommand(Robot robot) {

        this.robot = robot;

        // If you are going to use a subsystem in any of your methods, you
        // HAVE to uncomment the corresponding line below.
        addRequirements(robot.swerveDrive);
        addRequirements(robot.swerveDrive);
        // addRequirements(robot.hand);
        // addRequirements(robot.arm);
        addRequirements(robot.vision);
    }

    @Override
    public void initialize() {
        AHRS navx = new AHRS();
        double northAngle = 0;
        double eastAngle = 90;
        double southAngle = 180;
        double westAngle = 270;
        //going to the west Angle
        
        angleInitial = navx.getYaw();
        // REPLACE ME with real logic that actually does something

    }

    @Override
    public void execute() {
        AHRS navx = new AHRS();
        currentAngle = navx.getYaw();
        if (currentAngle > -90.50 && currentAngle < -89.50){
           robot.swerveDrive.drive(0, 0, 0, true);
           this.isFinished();
        } else if (angleInitial > -90 && angleInitial > 90){
           robot.swerveDrive.drive(0, 0 , Math.toRadians(-15), true);
        } else {
           robot.swerveDrive.drive(0, 0,Math.toRadians(15), true );
        }

        // REPLACE ME with real logic that actually does something
    }

    @Override
    public boolean isFinished() {
        // REPLACE ME with real logic that returns true only when your command is finished running
        return true;
    }
}
