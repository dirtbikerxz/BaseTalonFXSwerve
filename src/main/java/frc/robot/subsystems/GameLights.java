package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Spark;


public class GamerLight {

    public class Blinkin {

        // TODO: Spark ID
        Spark blinkin = new Spark(0);

        /* FOR FUTURE PURPOSES
         *  If needing to press a button on the controller, to change a LED LIGHT
         */

        // Joystick driverController = new Joystick(0);

    public Blinkin() {
    }
    
    // public void NAME() {
    //   blinkin.set(COLOR);
    // }

    /*  IF NEED COLOR DURING DRIVING
    public void driveLight() {
        blinkin.set(0.69);
      } */

    /* IF NEED COLOR DURING BUTTON PRESS 
     public void buttonLight() {
         blinkin.set(0.69);
       } */

    public void idleLight() {
      blinkin.set(0.85);
    }

    /* FOR FUTURE PURPOSES (IF YOU NEED A COLOR DURING DRIVING)
    Set name if using, set button id (X)

    public void DRIVING() {

        if(driverController.getRawButton(X)){
        // driveLight();
    }

    else if(driverController.getRawButton(X)){
         buttonLight();
    }
    */

    else {
        idleLight();
    }

         }
    }