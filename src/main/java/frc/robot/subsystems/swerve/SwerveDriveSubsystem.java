package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModule [] swerveModules;
    private final AHRS navx;
    private SwerveDriveKinematics kinematics;
    private boolean robotRelative;

    public SwerveDriveSubsystem() {

        navx = new AHRS(Port.kUSB);
        zeroGyro();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConfig.frontLeft),
            new SwerveModule(1, SwerveConfig.frontRight),
            new SwerveModule(2, SwerveConfig.backLeft),
            new SwerveModule(3, SwerveConfig.backRight)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

    }

    public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    public void setRobotRelative(boolean robotRelative) {
        this.robotRelative = robotRelative;
    }

    public void stop() {
        drive(0, 0, 0, 0.0);
    }

    public void drive(double vx, double vy, double vomega, double maxSpeed) {
        ChassisSpeeds speeds = robotRelative
                ? new ChassisSpeeds(vx, vy, vomega)
                : ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vomega, getYaw());
        drive(speeds, maxSpeed);
    }

    public void drive(ChassisSpeeds speeds, double maxSpeed) {
        SwerveModuleState [] moduleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates, maxSpeed);
    }    

    public void setModuleStates(SwerveModuleState [] desiredStates, double maxSpeed) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        for (SwerveModule module : swerveModules){
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }    

    public void zeroGyro() {
        navx.zeroYaw();
    }

    public Rotation2d getYaw() {
        if (navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(-navx.getFusedHeading());
        }
        return Rotation2d.fromDegrees(-(navx.getYaw())+0);
    }

    public void resetModulesToAbsolute(){
        for (SwerveModule module : swerveModules){
            module.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
        SmartDashboard.putBoolean("Mag Cal?", navx.isMagnetometerCalibrated());

        for(SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}