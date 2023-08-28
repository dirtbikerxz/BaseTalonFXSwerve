package frc.lib.math;

public class Conversions {

    public static double getOutputShaftRotations(double inputRotations, double gearRatio) {
        return inputRotations / gearRatio;
    }

    public static double getInputShaftRotations(double outputRotations, double gearRatio) {
        return outputRotations * gearRatio;
    }

}