package frc.robot;

public class RobotMode {
    
    public enum ModeOptions {
        CONE,
        CUBE;
    }

    public ModeOptions mode = ModeOptions.CUBE;

    public void SetMode(ModeOptions selectedMode) {

        mode = selectedMode;
    }

}
