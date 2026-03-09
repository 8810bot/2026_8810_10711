// Copyright 2026
package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for hopper sensors and servo controls. */
public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = false;
    public boolean canRange1Connected = false;
    public boolean canRange2Connected = false;
    public boolean servoHubConnected = false;

    /** CANrange 1 distance in meters (best-effort, 0 if unknown). */
    public double distance1Meters = 0.0;
    /** CANrange 2 distance in meters (best-effort, 0 if unknown). */
    public double distance2Meters = 0.0;

    /** Raw detection booleans computed from distance thresholds. */
    public boolean detected1 = false;

    public boolean detected2 = false;

    /** Last commanded servo positions in WPILib normalized range [0, 1]. */
    public double servo1Position = 0.0;

    public double servo2Position = 0.0;
    public double servo3Position = 0.0;
    public double servo4Position = 0.0;
    public double servo5Position = 0.0;
  }

  /** Refresh sensor readings and fill inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Set one servo position in normalized range [0, 1]. */
  public default void setServoPosition(int index, double position) {}
}
