package frc.robot.subsystems.swerve.falcon;

import frc.lib.math.GeometryUtils;
import frc.robot.constants.CTRESwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CTRESwerve extends SubsystemBase {
    
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public CTRESwerve() {
        gyro = new Pigeon2(CTRESwerveConstants.Swerve.pigeonID, "CANivore"); //"rio" (default), or the name of your CANivore
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, CTRESwerveConstants.Swerve.Mod0.constants),
                new SwerveModule(1, CTRESwerveConstants.Swerve.Mod1.constants),
                new SwerveModule(2, CTRESwerveConstants.Swerve.Mod2.constants),
                new SwerveModule(3, CTRESwerveConstants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.*/
        Timer.delay(1.0);
        resetModulesToAbsolute();
    }

    /*
     * correctForDynamics 
     * that takes in an instance of ChassisSpeeds as input and returns an 
     * updated instance of ChassisSpeeds after correcting for dynamics.
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02; 
        //This line declares a constant LOOP_TIME_S with a value of 0.02 seconds. 
        //This constant represents the time interval between two consecutive iterations of a loop or control cycle.
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S)); 
                //this line creates a new Pose2d object called futureRobotPose. 
                //It calculates the future position of the robot 
                //by multiplying the original speeds (vxMetersPerSecond, vyMetersPerSecond, and omegaRadiansPerSecond) by the loop time (LOOP_TIME_S). 
                //The resulting values are used to construct the Pose2d object.
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        //This line calculates the twist (linear and angular velocities) for the 
        //futureRobotPose by calling the log method from the GeometryUtils class, passing the futureRobotPose as an argument
        /*
        calculates the logarithm of a Pose2d object and returns a Twist2d object. 
        1. Get the rotation angle in radians from the Pose2d object:
        final double dtheta = transform.getRotation().getRadians();
        This line extracts the rotation angle in radians from the Pose2d object and assigns it to the variable `dtheta`.
        2. Calculate half of the rotation angle:
        final double half_dtheta = 0.5 * dtheta;
        This line calculates half of the rotation angle and assigns it to the variable `half_dtheta`.
        3. Calculate the value of `cos_minus_one`:
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        Here, the code calculates the difference between the cosine of the rotation angle and 1 and assigns it to the variable `cos_minus_one`.
        4. Calculate `halftheta_by_tan_of_halfdtheta` based on the value of `cos_minus_one`:
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
          halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
          halftheta_by_tan_of_halfdtheta =
              -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }
        This block of code checks the value of `cos_minus_one` against some small threshold `kEps` (defined elsewhere). If the absolute value of `cos_minus_one` is less than `kEps`, it sets `halftheta_by_tan_of_halfdtheta` to a specific formula. Otherwise, it uses a different formula to calculate `halftheta_by_tan_of_halfdtheta`.
        In the first case, if `cos_minus_one` is close to zero, it assumes the rotation is small and uses an approximation to calculate `halftheta_by_tan_of_halfdtheta`.
        In the second case, if `cos_minus_one` is not close to zero, it uses a more accurate formula to calculate `halftheta_by_tan_of_halfdtheta`.
        5. Calculate the translation part:
        final Translation2d translation_part =
            transform
                .getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        This line calculates the translation part of the Twist2d object. It first gets the translation vector from the Pose2d object using `getTranslation()`. Then it rotates the translation vector by a new Rotation2d object created with `halftheta_by_tan_of_halfdtheta` and `-half_dtheta` as the rotation parameters.
        Create and return a Twist2d object:
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
        Finally, a new Twist2d object is created with the `x` and `y` components from the translation part, and the `dtheta` value is used as the rotation component. This Twist2d object is then returned.
        In summary, the code calculates the logarithm of a Pose2d object by extracting the rotation angle, performing some calculations based on the rotation angle, and rotating the translation part of the Pose2d object. The resulting translation and rotation components are then used to create a Twist2d object, which is returned as the result.
        and thats the simple version of 2nd order kinematics
         */
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
        //Here, a new ChassisSpeeds object called updatedSpeeds is created. The dx, dy, and dtheta components 
        //of the updatedSpeeds are set based on the corresponding components of twistForPose divided by the loop time (LOOP_TIME_S).
        //AKA 2nd order kinematics
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {        
        ChassisSpeeds desiredChassisSpeeds =
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        SwerveModuleState[] swerveModuleStates = CTRESwerveConstants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, CTRESwerveConstants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

   

    public void stopDrive() {
        drive(new Translation2d(0, 0), 0, false, true);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, CTRESwerveConstants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (CTRESwerveConstants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }


    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }


    public double gyroTemp(){
        return gyro.getTemp();
    }
    
    @Override
    public void periodic() {
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}