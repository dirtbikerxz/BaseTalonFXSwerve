package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.vision.AprilTagGrabber;

public class Vizbot extends TimedRobot {

    private AprilTagGrabber grabber;
    private int frames;

    @Override
    public void robotInit() {
        grabber = new AprilTagGrabber();
        grabber.start();
        frames = 0;
    }

    @Override
    public void robotPeriodic() {
        if (++frames == 50) {
            System.err.println("current tag = "+grabber.getCurrentTag());
            frames = 0;
        }
    }
    
}
