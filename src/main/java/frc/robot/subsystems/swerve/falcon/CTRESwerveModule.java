package frc.robot.subsystems.swerve.falcon;

import edu.wpi.first.math.geometry.Rotation2d;

public class CTRESwerveModule {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final String canBus;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     * @param canBus
     */
    public CTRESwerveModule(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, String canBus) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.canBus = canBus;
        this.angleOffset = angleOffset;
    }
}
