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

  // ---- 战壕危险区域 (Trench Danger Zones) ----
  /**
   * 4 个战壕区域的 AABB（Axis-Aligned Bounding Box）定义。
   *
   * <p>每个条目为 {xMin, xMax, yMin, yMax}，均为場地绝对坐标（米）。
   * 同时适用于红蓝两方，无需区分联盟。
   *
   * <pre>
   * 区域       X 范围         Y 范围
   * 蓝队下方   4.05 ~ 5.15   0.00 ~ 1.25
   * 蓝队上方   4.05 ~ 5.15   6.80 ~ 8.05
   * 红队上方  11.35 ~ 12.50  6.80 ~ 8.05
   * 红队下方  11.35 ~ 12.50  0.00 ~ 1.25
   * </pre>
   */
  public static final double[][] TRENCH_DANGER_ZONES = {
    {4.05, 5.15, 0.00, 1.25}, // 蓝队下方
    {4.05, 5.15, 6.80, 8.05}, // 蓝队上方
    {11.35, 12.50, 6.80, 8.05}, // 红队上方
    {11.35, 12.50, 0.00, 1.25}, // 红队下方
  };

  /**
   * 进入危险区的边界向内缩紧量（m）。
   *
   * <p>进入临界 = 边界 + HYSTERESIS_ENTER_MARGIN （円内）
   */
  public static final double TRENCH_HYSTERESIS_ENTER_MARGIN = 0.10;

  /**
   * 离开危险区的边界向外扩展量（m）。
   *
   * <p>退出临界 = 边界 - HYSTERESIS_EXIT_MARGIN （圆外）
   */
  public static final double TRENCH_HYSTERESIS_EXIT_MARGIN = 0.10;

  private HopperConstants() {}
}
