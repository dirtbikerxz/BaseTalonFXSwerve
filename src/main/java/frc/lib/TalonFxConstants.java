package frc.lib;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class TalonFxConstants {
    public final int deviceNumber;
    public final TalonFXConfiguration allConfigs;
    public final NeutralMode neutralMode;
    public final InvertType invertType;
    public final boolean slowStatusFrame;    
    
    /**
     * Constants to be used with LazyTalonFX Util
     * @param deviceNumber
     * @param allConfigs
     * @param neutralMode
     * @param invertType
     * @param slowStatusFrames
     */
    public TalonFxConstants(int deviceNumber, TalonFXConfiguration allConfigs, NeutralMode neutralMode, InvertType invertType, boolean slowStatusFrame) {
        this.deviceNumber = deviceNumber;
        this.allConfigs = allConfigs;
        this.neutralMode = neutralMode;
        this.invertType = invertType;
        this.slowStatusFrame = slowStatusFrame;
    }
}
