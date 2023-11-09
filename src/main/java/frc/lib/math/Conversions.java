package frc.lib.math;

public class Conversions {

    /**
     * @param encoderRot Encoder Position: (in Rotations)
     * @param gearRatio Gear Ratio between Encoder and Mechanism
     * @return Mechanism Position: (in Degrees)
     */
    public static double encoderToDegrees(double encoderRot, double gearRatio) {
        double mechRot = encoderRot / gearRatio;
        double mechDeg = mechRot * 360.0;
        return mechDeg;
    }

    /**
     * @param mechDeg Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Encoder and Mechanism
     * @return Encoder Rotation: (in Rotations)
     */
    public static double degreesToEncoder(double mechDeg, double gearRatio) {
        double mechRot = mechDeg / 360.0;
        double encoderRot = mechRot * gearRatio;
        return encoderRot;
    }

    /**
     * @param motorRot Motor Position: (in Rotations)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Mechanism Position: (in Degrees)
     */
    public static double talonToDegrees(double motorRot, double gearRatio) {
        double motorDeg = motorRot * 360.0;
        double mechDeg = motorDeg / gearRatio;
        return mechDeg;
    }

    /**
     * @param degrees Mechanism Position: (in Degrees)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Rotation: (in Rotations)
     */
    public static double degreesToTalon(double mechDeg, double gearRatio) {
        double motorDeg = mechDeg * gearRatio;
        double motorRotations = motorDeg / 360.0;
        return motorRotations;
    }    

    /**
     * @param motorRPS Motor Velocity: (in Rotations per Second)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Mechanism Velocity: (in Rotations per Minute)
     */
    public static double talonToRPM(double motorRPS, double gearRatio) {
        double motorRPM = motorRPS * 60.0;
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param mechRPM Mechanism Velocity: (in Rotations per Minute)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Motor Velocity: (in Rotations per Second)
     */
    public static double RPMToTalon(double mechRPM, double gearRatio) {
        double motorRPM = mechRPM * gearRatio;
        double motorRPS = motorRPM / 60.0;
        return motorRPS;
    }

    /**
     * @param motorRPS Motor Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double talonToMPS(double motorRPS, double circumference, double gearRatio){
        double wheelRPS = motorRPS / gearRatio;
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelRPM Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Motor Velocity: (in Rotations per Second)
     */
    public static double MPSToTalon(double wheelRPM, double circumference, double gearRatio){
        double wheelRPS = wheelRPM / circumference;
        double motorRPS = wheelRPS * gearRatio;
        return motorRPS;
    }

    /**
     * @param motorRot Motor Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Wheel Distance: (in Meters)
     */
    public static double talonToMeters(double motorRot, double circumference, double gearRatio){
        double wheelRotations = motorRot / gearRatio;
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @param gearRatio Gear Ratio between Motor and Wheel
     * @return Motor Position: (in Rotations)
     */
    public static double MetersToTalon(double wheelMeters, double circumference, double gearRatio){
        double wheelRotations = wheelMeters / circumference;
        double motorRotations = wheelRotations * gearRatio;
        return motorRotations;
    }
}