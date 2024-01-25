package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{

    private PhotonCamera limelight = new PhotonCamera("2531photonvision_Port_1182_Output_MJPEG_Server");

    double cameraHeight = Units.inchesToMeters(0);
    double targetHeight = Units.inchesToMeters(0);
    double cameraPitchRadians = Units.degreesToRadians(0);
    double goalRangeMeters = Units.feetToMeters(3); 

    public Vision() {

    }


    public double getDistanceMethod() {
        var result = limelight.getLatestResult();
        if (result.hasTargets()) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, targetHeight, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));

               
        }
        return 0;
    }

    public double getYaw() {
        var result = limelight.getLatestResult();

        if (result.hasTargets()) {
            result.getBestTarget().getYaw();
        }
        return 0;
    }

    // public Transform3d translationMethod() {
    //     var result = limelight.getLatestResult();

    //     if (result.hasTargets()) {
    //         return  result.getBestTarget().getBestCameraToTarget();
    //     }
    //     return new Transform3d();
    // }

}
