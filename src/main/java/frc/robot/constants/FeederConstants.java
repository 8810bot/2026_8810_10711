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

/** Feeder CAN IDs and tuning. Update these to match your robot wiring & tuning. */
public final class FeederConstants {
  // CAN IDs
  public static final int MOTOR_1_ID = 17;
  public static final int FOLLOWER_1_ID = 18;
  /** TODO: set to your second feeder leader CAN ID. */
  // public static final int MOTOR_2_ID = 0;
  // /** TODO: set to your second feeder follower CAN ID. */
  // public static final int FOLLOWER_2_ID = 0;
  // Motor direction
  public static final boolean INVERTED = false;
  /** Feeder follower alignment relative to leader. Use Opposed to run opposite direction. */
  public static final MotorAlignmentValue FOLLOWER_1_ALIGNMENT = MotorAlignmentValue.Opposed;

  public static final MotorAlignmentValue FOLLOWER_2_ALIGNMENT = MotorAlignmentValue.Opposed;

  // Gear ratio (motor rotations per mechanism rotation)
  public static final double SENSOR_TO_MECH_RATIO = 1.0;

  // Velocity closed-loop gains (Phoenix Slot0)
  public static final double KP = 12.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  public static final double KV = 0.;
  public static final double KS = 5;

  // Current limits (amps)
  public static final boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 30.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SEC = 1.0;

  public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 50.0;

  /** Default command slow reverse speed (RPS). Tune. */
  public static final double DEFAULT_REVERSE_RPS = 0.0;

  private FeederConstants() {}
}
