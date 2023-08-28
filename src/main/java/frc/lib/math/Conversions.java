package frc.lib.math;

public class Conversions {

    /**
     * Converts rotations of input shaft (eg the motor) to rotations of the output shaft (eg the wheel)
     * @param inputRotations  The number of rotations of the input shaft
     * @param gearRatio The gear reduction between the input and output shafts (eg 10:1 would be 10)
     * @return The number of rotations of the output shaft after the gear reduction
     */
    public static double getOutputShaftRotations(double inputRotations, double gearRatio) {
        return inputRotations / gearRatio;
    }

    /**
     * Converts rotations of output shaft (eg the wheel) to rotations of the input shaft (eg the motor)
     * @param outputRotations The number of rotations of the output shaft
     * @param gearRatio The gear reduction between the input and output shafts (eg 10:1 would be 10)
     * @return The number of rotations of the input shaft after the gear reduction
     */
    public static double getInputShaftRotations(double outputRotations, double gearRatio) {
        return outputRotations * gearRatio;
    }

}