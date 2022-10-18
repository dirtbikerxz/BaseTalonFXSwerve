package frc.lib.util;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSSwerveConstants {
    
    /** Swerve Drive Specialties - MK4i Module*/
    public static final class SDSMK4i{

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        
        public static final class driveGearRatios{
            /** 8.14 : 1 */
            public static final double L1 = (8.14 / 1.0);
            /** 6.75 : 1 */
            public static final double L2 = (6.75 / 1.0);
            /** 6.12 : 1 */
            public static final double L3 = (6.12 / 1.0);
        }

        /** (150 / 7) : 1 */
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

        public static final double angleKP = 0.3;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;
        public static final boolean canCoderInvert = false;
    }

    /** Swerve Drive Specialties - MK4 Module*/
    public static final class SDSMK4{

        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        
        public static final class driveGearRatios{
            /** 8.14 : 1 */
            public static final double L1 = (8.14 / 1.0);
            /** 6.75 : 1 */
            public static final double L2 = (6.75 / 1.0);
            /** 6.12 : 1 */
            public static final double L3 = (6.12 / 1.0);
            /** 5.14 : 1 */
            public static final double L4 = (5.14 / 1.0);
        }

        /** 12.8 : 1 */
        public static final double angleGearRatio = (12.8 / 1.0);

        public static final double angleKP = 0.0;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;
        public static final boolean canCoderInvert = false;
    }
}

  