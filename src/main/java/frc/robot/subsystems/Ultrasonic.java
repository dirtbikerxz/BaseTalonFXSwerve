package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ultrasonic extends SubsystemBase{
    
    private AnalogInput ultrasonicCal = new AnalogInput(0);

    public double getDistanceValue(){
        double currentDistanceCentimeters = ultrasonicCal.getAverageValue() * 0.125;
        if (currentDistanceCentimeters < 30){
            return 0;
        } else {
            return currentDistanceCentimeters;
        }
    }
}
