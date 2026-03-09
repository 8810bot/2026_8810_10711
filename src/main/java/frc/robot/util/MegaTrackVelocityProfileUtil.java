package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Utility math helpers for MegaTrackVelocityProfileCommand. */
public final class MegaTrackVelocityProfileUtil {
  private MegaTrackVelocityProfileUtil() {}

  /** Returns r(T) = g - p - T*v (all in field frame). */
  public static Translation2d compensatedVector(
      Translation2d targetTranslation,
      Translation2d shotOrigin,
      ChassisSpeeds vField,
      double tSec) {
    return targetTranslation
        .minus(shotOrigin)
        .minus(new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond).times(tSec));
  }

  /** Fixed-point solve for flight time T where T = flightTimeMap(|r(T)|). */
  public static double iterateFlightTimeSec(
      Translation2d targetTranslation,
      Translation2d shotOrigin,
      ChassisSpeeds vField,
      InterpolatingDoubleTreeMap flightTimeMap,
      double seedTSec,
      int maxIters,
      double epsTSec,
      double relax,
      double minTSec,
      double maxTSec) {
    double t = MathUtil.clamp(seedTSec, minTSec, maxTSec);
    if (!Double.isFinite(t)) {
      t =
          MathUtil.clamp(
              flightTimeMap.get(targetTranslation.minus(shotOrigin).getNorm()), minTSec, maxTSec);
    }

    for (int i = 0; i < maxIters; i++) {
      Translation2d r = compensatedVector(targetTranslation, shotOrigin, vField, t);
      double d = r.getNorm();
      double tNext = MathUtil.clamp(flightTimeMap.get(d), minTSec, maxTSec);
      double tNew = (1.0 - relax) * t + relax * tNext;
      if (Math.abs(tNew - t) < epsTSec) {
        return tNew;
      }
      t = tNew;
    }

    return t;
  }

  /**
   * Feedforward omega estimate to keep pointing at target while translating.
   *
   * <p>omega = (r x rDot) / |r|^2 where rDot = -v or -(v + T*a).
   */
  public static double omegaFeedforwardRadPerSec(
      Translation2d compensatedVector,
      ChassisSpeeds vField,
      Translation2d aField,
      double tSec,
      boolean useAccelInFf,
      double minDistMeters,
      double maxAbsOmegaRadPerSec) {
    double rx = compensatedVector.getX();
    double ry = compensatedVector.getY();

    double rdotx = -vField.vxMetersPerSecond;
    double rdoty = -vField.vyMetersPerSecond;
    if (useAccelInFf) {
      rdotx = -(vField.vxMetersPerSecond + tSec * aField.getX());
      rdoty = -(vField.vyMetersPerSecond + tSec * aField.getY());
    }

    double den = Math.max(rx * rx + ry * ry, minDistMeters * minDistMeters);
    double omega = (rx * rdoty - ry * rdotx) / den;
    return MathUtil.clamp(omega, -maxAbsOmegaRadPerSec, maxAbsOmegaRadPerSec);
  }

  /** One profile step for omega target using a trapezoid velocity profile. */
  public static TrapezoidProfile.State stepOmegaProfile(
      TrapezoidProfile.State previous,
      double targetOmegaRadPerSec,
      TrapezoidProfile.Constraints constraints,
      double dtSec) {
    TrapezoidProfile profile = new TrapezoidProfile(constraints);
    return profile.calculate(
        Math.max(0.0, dtSec), previous, new TrapezoidProfile.State(targetOmegaRadPerSec, 0.0));
  }
}
