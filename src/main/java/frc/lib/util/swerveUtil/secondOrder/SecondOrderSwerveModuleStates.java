package frc.lib.util.swerveUtil.secondOrder;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SecondOrderSwerveModuleStates {
    private SwerveModuleState[] swerveModuleStates;
    private double[] moduleTurnSpeeds;

    /**
     * Creates a new {@code SecondOrderSwerveModuleStates}
     *
     * @param swerveModuleStates array of swerve module states
     * @param moduleTurnSpeeds array of speeds at which the modules change heading in rad per sec
     */
    public SecondOrderSwerveModuleStates(
            SwerveModuleState[] swerveModuleStates,
            double[] moduleTurnSpeeds
    ){
       this.swerveModuleStates = swerveModuleStates;
       this.moduleTurnSpeeds = moduleTurnSpeeds;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        return swerveModuleStates;
    }
    public double[] getModuleTurnSpeeds(){
        return moduleTurnSpeeds;
    }
    public void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates){
        this.swerveModuleStates = swerveModuleStates;
    }
    public void setModuleTurnSpeeds(double[] moduleTurnSpeeds){
        this.moduleTurnSpeeds = moduleTurnSpeeds;
    }
}
