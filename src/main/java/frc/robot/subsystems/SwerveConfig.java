package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveConfig {
        
    public static class ModuleConfig {
        
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final Rotation2d angleOffset;

        public ModuleConfig(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
        }
    }
}
