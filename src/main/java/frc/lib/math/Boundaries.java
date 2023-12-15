package frc.lib.math;


public class Boundaries {

    public static double to180Boundaries(double degrees){
        degrees %= 360;
        if(degrees > 180){
            degrees -= 360;
        }else if(degrees < -180){
            degrees += 360;
        }
        return degrees;
    }

    public static double to360Boundaries(double degrees){
        degrees %= 360;
        degrees = (degrees < 0) ? (degrees + 360) : degrees; 
        return degrees;
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    /**
     * This function uses the deadband and checks all the variables passed.
     * If ANY are true it returns true.
     * 
     * @param deadband minimum value
     * @param vars the varables to see if they past the deadband
     * @return
     */
    public static boolean iterativeOrDeadband(double deadband, double... vars){
        for(double var:vars){
            if(Math.abs(var) > deadband){
                return true;
            }
        }
        return false;
    }

    /**
     * This function uses the deadband and checks all the variables passed.
     * If ALL are true it returns true.
     * 
     * @param deadband minimum value
     * @param vars the varables to see if they past the deadband
     * @return 
     */
    private static int andDeadbandHold = 0;
    public static boolean iterativeAndDeadband(double deadband, double... vars){
        for(double var:vars){
            if(Math.abs(var) > deadband){
                andDeadbandHold++;
            }
        }
        return (andDeadbandHold / vars.length >= deadband) ? true : false;
    }
    
    public static double deadband(double input, double deadband) {
        if (Math.abs(input) < deadband)
            return 0;
        return input;
    }
    
    public static boolean withinDeadband(double input, double deadband){
        return deadband(input, deadband) == 0 ? true : false;
    }
}