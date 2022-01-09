package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class Ultrasonic {
    
    private AnalogInput ultrasonicCal = new AnalogInput(0);

    public double getDistanceValue(){
        double currentDistanceCentimeters = ultrasonicCal.getAverageValue() * 0.125;
        return currentDistanceCentimeters;
    }
}
