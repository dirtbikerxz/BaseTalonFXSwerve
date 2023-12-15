package frc.robot;

public class States {

    public static enum DriveStates {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates {
        aligned, unAligned, normal
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;
}
