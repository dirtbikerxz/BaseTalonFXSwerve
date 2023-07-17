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
}