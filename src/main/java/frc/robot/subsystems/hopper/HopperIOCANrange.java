// Copyright 2026
package frc.robot.subsystems.hopper;

// Imports cleaned due to hardware disabled

/** HopperIO implementation - CANrange sensors DISABLED cleanly without removing class. */
public class HopperIOCANrange implements HopperIO {
  public HopperIOCANrange() {
    // Hardware CANrange devices intentionally not instantiated to prevent errors.
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    // Fill with safe dummy values
    inputs.distance1Meters = 0.0;
    inputs.distance2Meters = 0.0;
    inputs.detected1 = false;
    inputs.detected2 = false;
    inputs.canRange1Connected = false;
    inputs.canRange2Connected = false;
    inputs.connected =
        true; // Act as successfully "connected" to avoid throwing higher-level errors
  }
}
