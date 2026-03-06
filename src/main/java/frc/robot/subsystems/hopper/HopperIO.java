// Copyright 2026
package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** IO interface for hopper sensors and servo controls. */
public interface HopperIO {
  class HopperIOInputs implements LoggableInputs {
    public boolean connected = false;

    /** CANrange 1 distance in meters (best-effort, 0 if unknown). */
    public double distance1Meters = 0.0;
    /** CANrange 2 distance in meters (best-effort, 0 if unknown). */
    public double distance2Meters = 0.0;

    /** Raw detection booleans computed from distance thresholds. */
    public boolean detected1 = false;

    public boolean detected2 = false;

    /** Last commanded servo positions in WPILib normalized range [0, 1]. */
    public double servo1Position = 0.0;
    public double servo2Position = 0.0;
    public double servo3Position = 0.0;
    public double servo4Position = 0.0;
    public double servo5Position = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("Distance1Meters", distance1Meters);
      table.put("Distance2Meters", distance2Meters);
      table.put("Detected1", detected1);
      table.put("Detected2", detected2);
      table.put("Servo1Position", servo1Position);
      table.put("Servo2Position", servo2Position);
      table.put("Servo3Position", servo3Position);
      table.put("Servo4Position", servo4Position);
      table.put("Servo5Position", servo5Position);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      distance1Meters = table.get("Distance1Meters", distance1Meters);
      distance2Meters = table.get("Distance2Meters", distance2Meters);
      detected1 = table.get("Detected1", detected1);
      detected2 = table.get("Detected2", detected2);
      servo1Position = table.get("Servo1Position", servo1Position);
      servo2Position = table.get("Servo2Position", servo2Position);
      servo3Position = table.get("Servo3Position", servo3Position);
      servo4Position = table.get("Servo4Position", servo4Position);
      servo5Position = table.get("Servo5Position", servo5Position);
    }
  }

  /** Refresh sensor readings and fill inputs. */
  default void updateInputs(HopperIOInputs inputs) {}

  /** Set one servo position in normalized range [0, 1]. */
  default void setServoPosition(int index, double position) {}
}
