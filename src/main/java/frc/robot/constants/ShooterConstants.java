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
  public static final boolean FLYWHEEL_LEADER_INVERTED = true;

  public static final MotorAlignmentValue FLYWHEEL_1_FOLLOWER_INVERTED =
      MotorAlignmentValue.Aligned;
  public static final MotorAlignmentValue FLYWHEEL_2_FOLLOWER_INVERTED =
      MotorAlignmentValue.Opposed;

  // Gear ratios (motor rotations per mechanism rotation)
  public static final double FLYWHEEL_SENSOR_TO_MECH_RATIO = 30. / 24.;

  /** Flywheel/exit location relative to robot center (meters). +X forward, +Y left. */
  public static final double FLYWHEEL_OFFSET_X_METERS = 0.0;

  public static final double FLYWHEEL_OFFSET_Y_METERS = 0.0;

  // Flywheel closed-loop gains (Phoenix Slot0)
  // Shooter 1
  public static final double FLYWHEEL_1_KP = 8;
  public static final double FLYWHEEL_1_KI = 0.7;
  public static final double FLYWHEEL_1_KD = 0.15;
  public static final double FLYWHEEL_1_KV = 0.0;
  public static final double FLYWHEEL_1_KS = 10;

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

  /** Flywheel control state-machine tuning (startup/hold/ball/recovery). */
  public static final class FlywheelControlConstants {
    /** Enter HOLD when within this band of the setpoint (RPS). */
    public static final double STARTUP_ENTER_HOLD_BAND_RPS = 1.0;
    /** Enter RECOVERY when speed drops below setpoint by this band (RPS). */
    public static final double RECOVERY_DROP_BAND_RPS = 3.0;
    /** Exit RECOVERY when speed is within this band of setpoint (RPS). */
    public static final double RECOVERY_EXIT_BAND_RPS = 1.5;

    /** Torque-current during ball contact (amps). */
    public static final double BALL_TORQUE_CURRENT_AMPS = 60.0;
    /** Torque-current feedforward used in HOLD (amps). */
    public static final double HOLD_TORQUE_FF_AMPS = 0.0;

    /** Minimum dwell time in each state (sec) to prevent chatter. */
    public static final double MODE_MIN_DWELL_SEC = 0.06;
    /** Maximum time to stay in BALL if no sensor clears (sec). */
    public static final double BALL_PHASE_TIMEOUT_SEC = 0.12;
    /** Duty cycle used for STARTUP/RECOVERY boost (-1.0 to 1.0). */
    public static final double BOOST_DUTY_CYCLE = 1.0;

    private FlywheelControlConstants() {}
  }

  private ShooterConstants() {}
}
