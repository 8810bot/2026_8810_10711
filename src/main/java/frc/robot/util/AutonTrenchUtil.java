package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/** Helper math and target selection for AutonTrench command. */
public final class AutonTrenchUtil {
  private static final double FIELD_MID_Y_METERS = Constants.FieldConstants.FIELD_CENTER.getY();
  private static final double DOWN_TRENCH_Y_METERS = findExtremeTrenchY(false);
  private static final double UP_TRENCH_Y_METERS = findExtremeTrenchY(true);

  private AutonTrenchUtil() {}

  public static double selectYTarget(double robotY) {
    if (robotY < FIELD_MID_Y_METERS) {
      return DOWN_TRENCH_Y_METERS;
    } else if (robotY > FIELD_MID_Y_METERS) {
      return UP_TRENCH_Y_METERS;
    }
    return FIELD_MID_Y_METERS;
  }

  public static double selectRotationTarget(double robotYawRad) {
    if (robotYawRad < (Math.PI / 2.0) && robotYawRad > -(Math.PI / 2.0)) {
      return 0.0;
    }
    return Math.PI;
  }

  public static double clampYSpeed(double ySpeedMetersPerSec) {
    return MathUtil.clamp(ySpeedMetersPerSec, -2.0, 2.0);
  }

  private static double findExtremeTrenchY(boolean findMax) {
    double[] ys =
        java.util.Arrays.stream(Constants.FieldConstants.TRENCH_CENTERS)
            .mapToDouble(center -> center.getY())
            .toArray();

    double value = ys[0];
    for (int i = 1; i < ys.length; i++) {
      if (findMax) {
        value = Math.max(value, ys[i]);
      } else {
        value = Math.min(value, ys[i]);
      }
    }
    return value;
  }
}
