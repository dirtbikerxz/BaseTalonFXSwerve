package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.other.Vision;
import frc.robot.other.Ultrasonic;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Ultrasonic ultrasonic;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private Vision vision;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Vision vision, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.vision = new Vision();

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        // System.out.println("Excetuting!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        vision.update();
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        if(controller.getRawButton(XboxController.Button.kX.value) && vision.getTargetFound()){   
            // System.out.println("pressed");
            rotation = vision.getAimValue();
            System.out.println("Aligning!!!!!!!!!!!!!!!!!!!!!");
        } else {
            rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        }

        System.out.println(rotation);


        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        // translation = new Translation2d(0, 0);

        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        
    }

    public void allign(){

        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        System.out.println("pressed");
        rotation = vision.getAimValue();
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

    }
}
