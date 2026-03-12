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

/** Intake CAN IDs and tuning. Update these to match your robot wiring. */
public final class IntakeConstants {
  // CAN IDs
  public static final int LEADER_MOTOR_ID = 22;
  /** TODO: set to your deploy/arm motor CAN ID (收放电机). */
  public static final int DEPLOY_MOTOR_ID = 23;

  // Motor directions
  public static final boolean LEADER_INVERTED = true;
  public static final boolean DEPLOY_INVERTED = true;

  // ---------------- Roller (吸球滚轮) ----------------
  /** Rollers voltage for intaking (volts). */
  public static final double ROLLER_INTAKE_VOLTS = 8;
  /** Rollers voltage for stopping (volts). */
  public static final double ROLLER_STOP_VOLTS = 0.;
  // Current limits (amps)
  public static final boolean ROLLER_ENABLE_SUPPLY_CURRENT_LIMIT = true;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
  public static final double ROLLER_SUPPLY_CURRENT_LOWER_LIMIT_AMPS = 30.0;
  public static final double ROLLER_SUPPLY_CURRENT_LOWER_TIME_SEC = 1.0;
  public static final boolean ROLLER_ENABLE_STATOR_CURRENT_LIMIT = true;
  public static final double ROLLER_STATOR_CURRENT_LIMIT_AMPS = 80.0;

  // ---------------- Deploy (收放摆臂) ----------------
  /** Sensor-to-mechanism ratio for deploy motor (motor rotations per mechanism rotation). */
  public static final double DEPLOY_SENSOR_TO_MECH_RATIO = 49.;

  /** Deploy position when stowed / hovering (mechanism rotations). */
  public static final double DEPLOY_POS_UP_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(23);

  public static final double DEPLOY_FLICK_UP = edu.wpi.first.math.util.Units.degreesToRotations(60);

  public static final double DEPLOY_POS_DEBUG_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(40);
  /** Deploy position when deployed down for intaking (mechanism rotations). */
  public static final double DEPLOY_POS_DOWN_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(10);

  public static final double FLIP_POS_UP = edu.wpi.first.math.util.Units.degreesToRotations(0);

  // Motion Magic (mechanism rotations/sec and rotations/sec^2)
  public static final double DEPLOY_MM_CRUISE_VELOCITY = 6.0;
  public static final double DEPLOY_MM_ACCELERATION = 4.0;
  public static final double DEPLOY_MM_JERK = 0.0;

  // Slot0 gains for MotionMagicTorqueCurrentFOC
  public static final double DEPLOY_KP = 4000;
  public static final double DEPLOY_KI = 0.0;
  public static final double DEPLOY_KD = 200;
  public static final double DEPLOY_KS = 0.0;
  public static final double DEPLOY_KG = 12;
  public static final double DEPLOY_KV = 0.0;
  public static final double DEPLOY_KA = 0.0;

  public static final double DEPLOY_PEAK_TORQUECURRENT_FORWARD = 120;
  public static final double DEPLOY_PEAK_TORQUECURRENT_REVERSE = -60;

  // Flick/backfeed behavior (间歇性收放拨球) - simple oscillation
  /** Max time allowed to travel toward flick up target (seconds). */
  public static final double FLICK_TRAVEL_UP_SEC = 0.7;
  /** Max time allowed to travel toward flick down target (seconds). */
  public static final double FLICK_TRAVEL_DOWN_SEC = 0.7;
  /** Hold time at flick up target once reached (seconds). */
  public static final double FLICK_HOLD_UP_SEC = 0.2;
  /** Hold time at flick down target once reached (seconds). */
  public static final double FLICK_HOLD_DOWN_SEC = 0.2;
  /** Position tolerance for considering deploy at target (mechanism rotations). */
  public static final double FLICK_POS_TOLERANCE_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(2.0);
  /** In flick mode, rollers voltage (volts). TODO tune (can be 0 or slight reverse). */
  public static final double FLICK_ROLLER_VOLTS = 5.0;

  // Shot-linked stow (射球时：根据射球数量"越收越回")
  /**
   * Extra stow amount per shot (mechanism rotations/shot).
   *
   * <p>Sign depends on your mechanism. If "more stowed" is a smaller rotation, use negative.
   */
  public static final double SHOOT_STOW_EXTRA_PER_SHOT_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(1.7);
  /** Clamp the total extra stow (mechanism rotations). */
  public static final double SHOOT_STOW_EXTRA_MIN_ROT = -0.2;

  public static final double SHOOT_STOW_EXTRA_MAX_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(45);

  // Homing (电流检测归零)
  /** 进入 HOMING 后等待此时间 (秒) 再检测堵转，避免启动电流误判。 */
  public static final double HOMING_SETTLE_SEC = 0.15;
  /** 归零时驱动电压 (V)。负值 = 向下运动到止点。 */
  public static final double HOMING_VOLTAGE = -1.5;
  /** 堵转判定电流阈值 (A)。 */
  public static final double HOMING_CURRENT_THRESHOLD = 5.0;
  /** 堵转判定转速阈值 (mechanism rot/s)。 */
  public static final double HOMING_VELOCITY_THRESHOLD = 0.02;
  /** 归零时重设的位置（机械止点角度，mechanism rotations）。 */
  public static final double HOMING_RESET_POS_ROT =
      edu.wpi.first.math.util.Units.degreesToRotations(-5);

  private IntakeConstants() {}
}
