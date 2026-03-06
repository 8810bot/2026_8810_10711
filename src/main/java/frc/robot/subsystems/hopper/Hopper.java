// Copyright 2026
package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper subsystem, including piece detection and servo interaction helpers.
 *
 * <p>Uses two CANrange sensors and a Debouncer to determine whether the hopper is "full".
 */
public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

  private final Debouncer fullDebouncer =
      new Debouncer(HopperConstants.FULL_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

  private boolean full = false;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    boolean fullRaw = inputs.detected1 && inputs.detected2;
    full = fullDebouncer.calculate(fullRaw);

    Logger.recordOutput("Hopper/FullRaw", fullRaw);
    Logger.recordOutput("Hopper/Full", full);
  }

  /** True if hopper is full (debounced). */
  public boolean isFull() {
    return full;
  }

  /** True if the CANrange sensors are healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Set one servo position by index [0..4]. */
  public void setServoPosition(int index, double position) {
    io.setServoPosition(index, position);
    Logger.recordOutput("Hopper/Servo" + (index + 1) + "Setpoint", position);
  }

  /** Command: move servos 1..4 to 0 and servo 5 to 0.5/3 in sequence. */
  public Command runServoDeploySequence() {
    return Commands.sequence(
            Commands.runOnce(() -> setServoPosition(0, 0.0), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(1, 0.0), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(2, 0.0), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(3, 0.0), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(4, 0.5 / 3.0), this))
        .withName("HopperServoDeploySequence");
  }

  /** Command: restore servos in reverse order to center (0.5). */
  public Command runServoRestoreSequence() {
    return Commands.sequence(
            Commands.runOnce(() -> setServoPosition(4, 0.5), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(3, 0.5), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(2, 0.5), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(1, 0.5), this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setServoPosition(0, 0.5), this))
        .withName("HopperServoRestoreSequence");
  }
}
