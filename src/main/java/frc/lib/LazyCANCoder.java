package frc.lib;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

/**
 * Thin CANCoder wrapper to make setup easier.
 */
public class LazyCANCoder extends CANCoder {

    public LazyCANCoder(int deviceNumber, CANCoderConfiguration allConfigs){
        super(deviceNumber, "canivore");
        super.configFactoryDefault();
        super.configAllSettings(allConfigs);
    }
    
}
