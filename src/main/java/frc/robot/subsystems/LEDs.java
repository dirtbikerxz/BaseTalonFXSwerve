package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.robot.Constants;

public class LEDs {
   
    
    CANdle candle = new CANdle(Constants.CANDLE_PORT);
    
    public LEDs() {

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        candle.configAllSettings(config);

    }

    public void rainbow() {

        
        candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white
            
        // create a rainbow animation:
        // - max brightness
        // - half speed
        // - 64 LEDs
        RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 8);
        candle.animate(rainbowAnim);
    }

    public void setColor(int red, int green, int blue) {

        candle.setLEDs(red, green, blue);
    }


    public void purple() {

        candle.setLEDs(138,43,226);

    }

    public void yellow() {

        candle.setLEDs(255,255,0);
    }

    public void clear() {
        candle.setLEDs(0,0,0);
    }
    
}
