package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    ShuffleboardTab tab; 
    ShuffleboardLayout flList;
    ShuffleboardLayout frList;
    ShuffleboardLayout rlList;
    ShuffleboardLayout rrList;
    
  //  public Pigeon2 gyro;
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    public Swerve() {
        tab = Shuffleboard.getTab("Drivetrain");
        frList = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0);
        flList = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(2, 0);
        rlList = tab.getLayout("Rear Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(4, 0);
        rlList = tab.getLayout("Rear Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(6, 0);
        

        tab.add(m_navx);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
    m_navx.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_navx.getYaw()) : Rotation2d.fromDegrees(m_navx.getYaw());
    }

    public void updateDashboard(){

        flList.add("Cancoder",mSwerveMods[0].getCanCoder().getDegrees());
        flList.add("Position",mSwerveMods[0].getPosition().angle.getDegrees());
        flList.add("Velocity",mSwerveMods[0].getState().speedMetersPerSecond);
        
        frList.add("Cancoder",mSwerveMods[1].getCanCoder().getDegrees());
        frList.add("Position",mSwerveMods[1].getPosition().angle.getDegrees());
        frList.add("Velocity",mSwerveMods[1].getState().speedMetersPerSecond);
        
        rlList.add("Cancoder",mSwerveMods[2].getCanCoder().getDegrees());
        rlList.add("Position",mSwerveMods[2].getPosition().angle.getDegrees());
        rlList.add("Velocity",mSwerveMods[2].getState().speedMetersPerSecond);
        
        rrList.add("Cancoder",mSwerveMods[3].getCanCoder().getDegrees());
        rrList.add("Position",mSwerveMods[3].getPosition().angle.getDegrees());
        rrList.add("Velocity",mSwerveMods[3].getState().speedMetersPerSecond);

        // odometryXEntry = tab.add("ODO_X", 0.0)
        //     .withPosition(8, 0)
        //     .withSize(1, 1)
        //     .getEntry();
        // odometryYEntry = tab.add("ODO_Y", 0.0)
        //     .withPosition(8, 1)
        //     .withSize(1, 1)
        //     .getEntry();
        // odometryAngleEntry = tab.add("ODO_ANGLE", 0.0)
        //     .withPosition(8, 2)
        //     .withSize(1, 1)
        //     .getEntry();
    }
    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  
        updateDashboard();
    }
}