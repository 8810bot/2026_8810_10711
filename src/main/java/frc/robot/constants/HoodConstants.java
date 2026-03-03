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

/** Shared hood (angle adjustment) constants. Used by {@code Hood} subsystem. */
public final class HoodConstants {
  // CAN IDs
  public static final int MOTOR_ID = 21;

  // Motor direction
  public static final boolean INVERTED = true;

  // Gear ratio (motor rotations per mechanism rotation)
  public static final double SENSOR_TO_MECH_RATIO = 160. / 9. * 34. / 20. * 26. / 12.;

  // Motion Magic (mechanism rotations/sec and rotations/sec^2)
  public static final double MM_CRUISE_VELOCITY = 0.5;
  public static final double MM_ACCELERATION = 1.0;
  public static final double MM_JERK = 0.0;

  // Closed-loop gains (Phoenix Slot0)
  public static final double KP = 2300;
  public static final double KI = 0.0;
  public static final double KD = 230.0;
  public static final double KS = 0.0;
  public static final double KG = 2.0;
  public static final double KV = 0.0;
  public static final double KA = 0.0;

  private HoodConstants() {}
}
