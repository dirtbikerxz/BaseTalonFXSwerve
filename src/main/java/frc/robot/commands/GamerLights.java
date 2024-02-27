package frc.robot.commands;
package org.firstinspires.ftc.teamcode;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class LEDLightsTutorial extends OpMode {

  RevBlinkinLedDriver lights;

  // If using pattern, tempo
  int temp = 1

    public void init()
    {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, deviceName : "lights");

        //Starting color
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

    }

    public void loop() {

        if (temp == 1) {
            restartStartTime();
            temp = 2;
        }

        if (time >= 5 && <=15) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        }  
        if (time >= 15 && <=25) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } 
        
        else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
    }
}