package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic {
    
    private final AnalogInput ultrasonic = new AnalogInput(0);
    
    double currentDistanceCentimeters = ultrasonic.getAverageValue() * 0.125;

        if(currentDistanceCentimeters <= 30){
            System.out.println("Under 30cm");
        } else {
            System.out.println(currentDistanceCentimeters + "cm");
        }
}
