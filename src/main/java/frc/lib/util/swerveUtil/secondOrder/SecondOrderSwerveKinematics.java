package frc.lib.util.swerveUtil.secondOrder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

import static edu.wpi.first.math.Nat.N1;
import static edu.wpi.first.math.Nat.N2;
import static edu.wpi.first.math.Nat.N3;
import static edu.wpi.first.math.Nat.N4;


public class SecondOrderSwerveKinematics {

    private Translation2d[] moduleLocations = new Translation2d[4]; //Module location vectors from the center of rotation (center of the robot)

    /**
     * Creates a new {@code SecondOrderSwerveKinematics}
     *
     * @param frontLeftLocation Location of front left swerve module in meters as Translation2d object
     * @param frontRightLocation Location of front right swerve module in meters
     * @param backLeftLocation Location of back left swerve module in meters
     * @param backRightLocation Location of back right swerve module in meters
     */
    public SecondOrderSwerveKinematics(
        Translation2d frontLeftLocation,
        Translation2d frontRightLocation,
        Translation2d backLeftLocation,
        Translation2d backRightLocation
    ){
     moduleLocations[0] = frontLeftLocation;
     moduleLocations[1] = frontRightLocation;
     moduleLocations[2] = backLeftLocation;
     moduleLocations[3] = backRightLocation;
    }

    /**
     * Convert chassis speed to states of individual modules using second order kinematics
     *
     * @param desiredSpeed desired translation and rotation speed of the robot
     * @param robotHeading heading of the robot relative to the field
     * @return SecondOrderSwerveModuleState[] array of the speed direction and turn speed of the swerve modules
     */
    public SecondOrderSwerveModuleStates toSwerveModuleState(ChassisSpeeds desiredSpeed, Rotation2d robotHeading){
        Matrix<N3, N1> firstOrderInputMatrix = new Matrix<>(N3(),N1());
        Matrix<N2, N3> firstOrderMatrix = new Matrix<>(N2(),N3());
        Matrix<N4, N1> secondOrderInputMatrix = new Matrix<>(N4(),N1());
        Matrix<N2, N4> secondOrderMatrix = new Matrix<>(N2(),N4());
        Matrix<N2, N2> rotationMatrix = new Matrix<>(N2(),N2());

        firstOrderInputMatrix.set(0,0, desiredSpeed.vxMetersPerSecond);
        firstOrderInputMatrix.set(1,0, desiredSpeed.vyMetersPerSecond);
        firstOrderInputMatrix.set(2,0, desiredSpeed.omegaRadiansPerSecond);

        secondOrderInputMatrix.set(2,0,Math.pow(desiredSpeed.omegaRadiansPerSecond, 2));

        firstOrderMatrix.set(0,0,1);
        firstOrderMatrix.set(1,1,1);

        secondOrderMatrix.set(0,0,1);
        secondOrderMatrix.set(1,1,1);

        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        double[] moduleTurnSpeeds = new double[4];

        for (int i = 0; i < 4; i ++){
            Rotation2d moduleAngle = new Rotation2d(Math.atan2(moduleLocations[i].getY(), moduleLocations[i].getX())); //Angle that the module location vector makes with respect to the robot
            Rotation2d moduleAngleFieldCentric = moduleAngle.plus(robotHeading); //Angle that the module location vector makes with respect to the field
            double moduleX = moduleLocations[i].getNorm() * Math.cos(moduleAngleFieldCentric.getRadians());
            double moduleY = moduleLocations[i].getNorm() * Math.sin(moduleAngleFieldCentric.getRadians());
            firstOrderMatrix.set(0,2,-moduleY); //-r_y
            firstOrderMatrix.set(1,2,moduleX); //r_x

            Matrix<N2, N1> firstOrderOutput = firstOrderMatrix.times(firstOrderInputMatrix);

            double moduleHeading = Math.atan2(firstOrderOutput.get(1,0), firstOrderOutput.get(0,0));
            double moduleSpeed = Math.sqrt(firstOrderOutput.elementPower(2).elementSum());

            secondOrderMatrix.set(0,2,-moduleX);
            secondOrderMatrix.set(0,3,-moduleY);
            secondOrderMatrix.set(1,2,-moduleY);
            secondOrderMatrix.set(1,3,moduleX);

            rotationMatrix.set(0,0,Math.cos(moduleHeading));
            rotationMatrix.set(0,1,Math.sin(moduleHeading));
            rotationMatrix.set(1,0,-Math.sin(moduleHeading));
            rotationMatrix.set(1,1,Math.cos(moduleHeading));

            Matrix<N2,N1> secondOrderOutput = rotationMatrix.times(secondOrderMatrix.times(secondOrderInputMatrix));

            swerveModuleStates[i] = new SwerveModuleState(moduleSpeed, new Rotation2d(moduleHeading).minus(robotHeading));
            moduleTurnSpeeds[i] = secondOrderOutput.get(1,0) / moduleSpeed - desiredSpeed.omegaRadiansPerSecond;
        }

        return new SecondOrderSwerveModuleStates(swerveModuleStates, moduleTurnSpeeds);
    }
}
