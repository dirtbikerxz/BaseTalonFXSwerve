package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.Vision;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Ultrasonic;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private Vision vision;

    private double ultrasonicDistance;

    private Ultrasonic ultrasonic = new Ultrasonic();


    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        vision = new Vision();

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        if(controller.getRawButton(2)){   
            System.out.println("pressed");
            rotation = vision.getAimValue();
        } else {
            rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        }
        ultrasonicDistance = ultrasonic.getDistanceValue();

        if(ultrasonicDistance >= 30){
            System.out.println(ultrasonicDistance);
        } else {
            System.out.println("Less than 30cm");
        }

        

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
