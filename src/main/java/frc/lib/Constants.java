package frc.lib;

public final class Constants extends doubleNeoConstants{


    public static final class Intake {
        public static final int intakeMotor = 32;
        public static final double intakeSpeed = 0.8;
    }

    public static final class Arm {
        public static final int armMotor = 33;
        public static final int armFollowerMotor = 34;
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double desiredShooterAngle = 30.0;
        public static final double desiredIntakeAngle = -30;
        public static final double armEncoderOffset = 0.0;

        public static final double minAngle = 31.0;
        public static final double maxAngle = 31.0;
    }

    public static final class Shooter {
        public static final int shooterMotor = 35;
        public static final int shooterFollowerMotor = 36;
        public static final int shooterIntakeMotor = 37;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double shooterIntakeSpeed = 0.8;
        public static final double shooterSpeed = 0.8;
    }


}
