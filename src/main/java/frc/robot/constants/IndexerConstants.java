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

/** Indexer CAN IDs and tuning. Update these to match your robot wiring & tuning. */
public final class IndexerConstants {
  public static final int MOTOR_1_ID = 19;
  /** TODO: set to your second indexer CAN ID. */
  public static final int MOTOR_2_ID = 20;

  public static final boolean INVERTED = true;

  // ---- IndexerUp ----
  /** IndexerUp 电机 CAN ID。TODO: 确认实际 CAN ID */
  public static final int INDEXER_UP_MOTOR_ID = 0;
  /** IndexerUp 减速比 (电机转 / 输出轴转)。1:2 表示电机转2圈输出转1圈。 */
  public static final double INDEXER_UP_SENSOR_TO_MECH_RATIO = 2.0;
  /** IndexerUp 电机是否反转 */
  public static final boolean INDEXER_UP_INVERTED = false;

  // Current limits (amps)
  public static final boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
  public static final double SUPPLY_CURRENT_LIMIT_AMPS = 50.0;
  public static final double SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 70.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SEC = 0.1;

  public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
  public static final double STATOR_CURRENT_LIMIT_AMPS = 120.0;

  /** Default command slow reverse voltage (volts). Tune. */
  public static final double DEFAULT_REVERSE_VOLTS = 0.0;

  private IndexerConstants() {}
}
