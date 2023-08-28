package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final String driveMotorBus;
    public final String angleMotorBus;
    public final String cancoderBus;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID CAN ID of the drive motor
     * @param angleMotorID CAN ID of the angle motor
     * @param canCoderID CAN ID of the cancoder
     * @param driveMotorBus CAN Bus of the drive motor
     * @param angleMotorBus CAN Bus of the angle motor
     * @param cancoderBus CAN Bus of the cancoder
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, String driveMotorBus, String angleMotorBus, String cancoderBus, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.driveMotorBus = driveMotorBus;
        this.angleMotorBus = angleMotorBus;
        this.cancoderBus = cancoderBus;
        this.angleOffset = angleOffset;
    }
}
