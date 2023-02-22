package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModule [] swerveModules;
    private final AHRS navx;
    private final Field2d field;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private boolean robotRelative;
    private double maxLinearSpeed;
    private double maxAngularSpeed;
    private double maxWheelSpeed;

    public SwerveDriveSubsystem() {

        navx = new AHRS(Port.kUSB);
        zeroGyro();

        field = new Field2d();
        SmartDashboard.putData("field", field);

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConfig.frontLeft),
            new SwerveModule(1, SwerveConfig.frontRight),
            new SwerveModule(2, SwerveConfig.backLeft),
            new SwerveModule(3, SwerveConfig.backRight)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with
         * inverting motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for
         * more info.
         */
        Timer.delay(1.0);
        for (SwerveModule module : swerveModules){
            module.resetToAbsolute();
        }
        
        setOrbitMode(false);
        maxLinearSpeed = SwerveConfig.defaultMaxLinearSpeed;
        maxAngularSpeed = SwerveConfig.defaultMaxAngularSpeed;
        maxWheelSpeed = SwerveConfig.defaultMaxWheelSpeed;
        robotRelative = false;

        SmartDashboard.putData("Swerve Gyro", builder -> {
            builder.addBooleanProperty("Calibrated", navx::isMagnetometerCalibrated, null);
            builder.addDoubleProperty("Pitch", () -> getPitch(), null);
            builder.addDoubleProperty("Yaw", () -> getYaw().getDegrees(), null);
        });
        SmartDashboard.putData("Swerve Max Speeds", builder -> {
            builder.addDoubleProperty("Angular", () -> maxAngularSpeed, val -> maxAngularSpeed = val);
            builder.addDoubleProperty("Linear", () -> maxLinearSpeed, val -> maxLinearSpeed = val);
            builder.addDoubleProperty("Wheel", () -> maxWheelSpeed, val -> maxWheelSpeed = val);
        });
        SmartDashboard.putData("Swerve Odometry", builder -> {
            builder.addDoubleProperty("Heading", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
            builder.addDoubleProperty("X", () -> Units.metersToFeet(odometry.getPoseMeters().getX()), null);
            builder.addDoubleProperty("Y", () -> Units.metersToFeet(odometry.getPoseMeters().getY()), null);
        });
    }

    public SwerveModulePosition [] getModulePositions() {
        SwerveModulePosition [] positions = new SwerveModulePosition[4];
        for (int i=0; i<4; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    public void setOrbitMode(boolean orbit) {

        SwerveDriveKinematics targetKinematics = orbit
            ? SwerveConfig.orbitKinematics
            : SwerveConfig.defaultKinematics;
        if (kinematics == targetKinematics) {
            return;
        }

        if (odometry == null) {
            odometry = new SwerveDriveOdometry(
                    kinematics,
                    getYaw(),
                    getModulePositions());
        } else {
            Pose2d currentPose = odometry.getPoseMeters();
            odometry = new SwerveDriveOdometry(
                    kinematics,
                    getYaw(),
                    getModulePositions(),
                    currentPose);
        }
        this.kinematics = targetKinematics;
    }

    public void setRobotRelative(boolean robotRelative) {
        this.robotRelative = robotRelative;
    }

    public void setTurboMode(boolean turbo) {
        if (turbo) {
            maxLinearSpeed = SwerveConfig.defaultMaxLinearSpeed * SwerveConfig.turboFactor;
            maxAngularSpeed = SwerveConfig.defaultMaxAngularSpeed * SwerveConfig.turboFactor;
            maxWheelSpeed = SwerveConfig.defaultMaxWheelSpeed * SwerveConfig.turboFactor;
        } else {
            maxLinearSpeed = SwerveConfig.defaultMaxLinearSpeed;
            maxAngularSpeed = SwerveConfig.defaultMaxAngularSpeed;
            maxWheelSpeed = SwerveConfig.defaultMaxWheelSpeed;
        }
    }

    /**
     * Stops the drive entirely
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Runs the drive at a percentage of max speed
     */
    public void drive(double percentX, double percentY, double percentOmega) {

        double vx = percentX * maxLinearSpeed;
        double vy = percentY * maxLinearSpeed;
        double vomega = percentOmega * maxAngularSpeed;

        ChassisSpeeds speeds = robotRelative
                ? new ChassisSpeeds(vx, vy, vomega)
                : ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vomega, getYaw());
        drive(speeds);
    }

    /**
     * Runs the drive at a set of specified speeds
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState [] moduleStates = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(moduleStates);
    }

    /**
     * Directly sets module state for each wheel
     */
    public void setModuleStates(SwerveModuleState [] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxWheelSpeed);
        for (SwerveModule module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleNumber], false);
        }
    }    

    public void zeroGyro() {
        navx.zeroYaw();
    }

    public double getPitch() {
        return navx.getPitch();
    }

    public Rotation2d getYaw() {
        if (navx.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(-navx.getFusedHeading());
        }
        return Rotation2d.fromDegrees(-(navx.getYaw())+0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        field.setRobotPose(odometry.getPoseMeters());
    }
}