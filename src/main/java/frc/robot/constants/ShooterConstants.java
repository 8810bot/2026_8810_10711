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

import com.ctre.phoenix6.signals.MotorAlignmentValue;

/** Shooter CAN IDs and tuning. Update these to match your robot wiring & tuning. */
public final class ShooterConstants {
  // CAN IDs
  public static final int FLYWHEEL_1_LEADER_ID = 13;
  public static final int FLYWHEEL_1_FOLLOWER_ID = 14;
  public static final int FLYWHEEL_2_LEADER_ID = 15;
  public static final int FLYWHEEL_2_FOLLOWER_ID = 16;

  // Motor directions
  public static final boolean FLYWHEEL_LEADER_INVERTED = false;

  public static final MotorAlignmentValue FLYWHEEL_1_FOLLOWER_INVERTED =
      MotorAlignmentValue.Aligned;
  public static final MotorAlignmentValue FLYWHEEL_2_FOLLOWER_INVERTED =
      MotorAlignmentValue.Aligned;

  // Gear ratios (motor rotations per mechanism rotation)
  public static final double FLYWHEEL_SENSOR_TO_MECH_RATIO = 30. / 24.;

  /** Flywheel/exit location relative to robot center (meters). +X forward, +Y left. */
  public static final double FLYWHEEL_OFFSET_X_METERS = -0.15;

  public static final double FLYWHEEL_OFFSET_Y_METERS = 0.0;

  // Flywheel closed-loop gains (Phoenix Slot0)
  // Shooter 1
  public static final double FLYWHEEL_1_KP = 15;
  public static final double FLYWHEEL_1_KI = 0.0;
  public static final double FLYWHEEL_1_KD = 0.0;
  public static final double FLYWHEEL_1_KV = 0.34;
  public static final double FLYWHEEL_1_KS = 12;

  // Shooter 2 (default same as shooter 1; tune independently)
  public static final double FLYWHEEL_2_KP = 15;
  public static final double FLYWHEEL_2_KI = 0.0;
  public static final double FLYWHEEL_2_KD = 0.0;
  public static final double FLYWHEEL_2_KV = 0.34;
  public static final double FLYWHEEL_2_KS = 12;

  // Current limits (amps). Tune to protect wiring/breakers.
  public static final boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 40.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SEC = 1.0;

  public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;

  // Shot counting (based on flywheel current drop)
  /** If max(leader,follower) stator current is below this, treat as "low current". Tune. */
  public static final double SHOT_COUNT_LOW_CURRENT_THRESHOLD_AMPS = 30.0;
  /** Low-current debounce time (sec). */
  public static final double SHOT_COUNT_DEBOUNCE_SEC = 0.005;
  /** Only count shots when setpoint is above this (RPS), to avoid counting at idle. */
  public static final double SHOT_COUNT_MIN_SETPOINT_RPS = 5.0;

  private ShooterConstants() {}
}
