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

/** Hopper detection constants (two CANrange sensors). */
public final class HopperConstants {
  /** TODO: set to your CANrange sensor IDs. */
  public static final int CANRANGE_1_ID = 0;

  public static final int CANRANGE_2_ID = 0;

  /** REV Servo Hub CAN device ID. */
  public static final int SERVO_HUB_CAN_ID = 3;

  /** Distance threshold (meters) for "piece present". Tune on your robot. */
  public static final double DETECTION_DISTANCE_METERS = 0.25;

  /** Debounce time (sec) for declaring hopper full / not full. */
  public static final double FULL_DEBOUNCE_SEC = 0.10;

  private HopperConstants() {}
}
