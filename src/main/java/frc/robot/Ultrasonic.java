package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic {
    
    private final AnalogInput ultrasonic = new AnalogInput(0);

    public double getDistanceValue(){
        double currentDistanceCentimeters = ultrasonic.getAverageValue() * 0.125;
        return currentDistanceCentimeters;
    }
}
