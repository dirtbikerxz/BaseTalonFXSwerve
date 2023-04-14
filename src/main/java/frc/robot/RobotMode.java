package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotMode {
    
    public enum ModeOptions {
        CONE,
        CUBE;
    }

    public enum StateOptions {
        LOW,
        MID,
        HIGH,
        SINGLE,
        DOUBLE,
        STOW;
    }

    public static ModeOptions mode = ModeOptions.CUBE;

    public static StateOptions state = StateOptions.STOW;

    public static void SetMode(ModeOptions selectedMode) {

        mode = selectedMode;

    }

    public static void SetState(StateOptions selectedState) {

        state = selectedState;
    }

    public static boolean IsCone() {

        return (mode == ModeOptions.CONE);
    }

}
