package frc.lib.util;

public class NeoSwerveModuleConstants {    
  
    public final int driveMotorChannel;
    public final int turningMotorChannel;
    public final int cancoderID;
    public final double angleOffset;
 

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public NeoSwerveModuleConstants(int driveMotorID,
        int turningMotorID,
        int cancoderID,
        double angleOffset) {

        this.driveMotorChannel = driveMotorID;
        this.turningMotorChannel = turningMotorID;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
    }

}