package frc.robot;

public class States {

    public static enum DriveStates {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static DriveStates driveState = DriveStates.standard;
}
