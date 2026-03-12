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

/** LED (CANdle) constants. */
public final class LEDConstants {
  /** TODO: set to your CANdle CAN ID. */
  public static final int CANDLE_ID = 60;

  /** First LED index to control. */
  public static final int LED_START_INDEX = 0;

  /** Number of LEDs to control starting from {@link #LED_START_INDEX}. */
  public static final int LED_COUNT = 8;

  private LEDConstants() {}
}
