package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class GeometryUtils {
  private static final double kEps = 1E-9;


  public static Pose2d exp(final Twist2d delta) {
    double sin_theta = Math.sin(delta.dtheta);
    double cos_theta = Math.cos(delta.dtheta);
    double s, c;
    if (Math.abs(delta.dtheta) < kEps) {
      s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
      c = .5 * delta.dtheta;
    } else {
      s = sin_theta / delta.dtheta;
      c = (1.0 - cos_theta) / delta.dtheta;
    }
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cos_theta, sin_theta));
  }

  public static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < kEps) {
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    } else {
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
    }
    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }
  /*
   * to explain this better
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
}