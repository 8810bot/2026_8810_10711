// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.constants;

/**
 * Drivetrain torque-current deadband tuning (applies when
 * DriveMotorClosedLoopOutput=TorqueCurrentFOC).
 */
public final class DrivetrainConstants {
  /**
   * Deadband for torque-current requests (same units as {@code TorqueCurrentFOC.withOutput}, amps).
   */
  public static final double TORQUE_CURRENT_DEADBAND_AMPS = 0.0;

  // --- Tilt detection ---
  /** Angle threshold (deg) to declare robot is tilted. */
  public static final double TILT_TRIP_DEG = 4.0;
  /** Angle threshold (deg) to clear tilted state (hysteresis). */
  public static final double TILT_CLEAR_DEG = 2.0;
  /** Debounce time (sec) for declaring tilted. */
  public static final double TILT_TRIP_DEBOUNCE_SEC = 0.10;
  /** Debounce time (sec) for clearing tilted. */
  public static final double TILT_CLEAR_DEBOUNCE_SEC = 0.10;

  private DrivetrainConstants() {}
}
