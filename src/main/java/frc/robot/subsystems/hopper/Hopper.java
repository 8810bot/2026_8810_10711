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
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public enum HopperTargetState {
    DOWN_STOW_STEP1,
    DOWN_STOW_STEP2,
    DOWN_STOW,
    UP_DEPLOY_STEP1,
    UP_DEPLOY_STEP2,
    UP_DEPLOY,
    MID_INTAKE
  }

  private HopperTargetState TargetState = HopperTargetState.UP_DEPLOY;
  private double stateStartTime = 0.0;
  private static final double SEQUENCE_DELAY_SEC = 0.2;

  private final Debouncer fullDebouncer =
      new Debouncer(HopperConstants.FULL_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

  private boolean full = false;

  public Hopper(HopperIO io) {
    this.io = io;
    init();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    applyTargetState();

    Logger.processInputs("Hopper", inputs);

    boolean fullRaw = inputs.detected1 && inputs.detected2;
    full = fullDebouncer.calculate(fullRaw);

    Logger.recordOutput("Hopper/FullRaw", fullRaw);
    Logger.recordOutput("Hopper/Full", full);
    Logger.recordOutput("Hopper/Connected", inputs.connected);
    Logger.recordOutput("Hopper/CANRange1Connected", inputs.canRange1Connected);
    Logger.recordOutput("Hopper/CANRange2Connected", inputs.canRange2Connected);
    Logger.recordOutput("Hopper/ServoHubConnected", inputs.servoHubConnected);
  }

  public void setTargetState(HopperTargetState state) {
    if (TargetState != state) {
      TargetState = state;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
  }

  public HopperTargetState getTargetState() {
    return TargetState;
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
    Logger.recordOutput("Hopper/Servo" + (index) + "Setpoint", position);
  }

  private void init() {
    setServoPosition(0, 0.0);
    setServoPosition(1, 0.0);
    setServoPosition(2, 0.0);
    setServoPosition(5, 0.0);
    ;
  }
  /** Apply the target state to the hopper servos. */
  private void applyTargetState() {
    double timeInState = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;

    switch (TargetState) {

        // ================= POV DOWN (全变 0.5) =================
      case DOWN_STOW_STEP1:
        // POV DOWN 步骤一：相反顺序，先设置通道3(idx 2)为0.35
        setServoPosition(2, 0.4);
        setServoPosition(0, 0.45);

        // 延时条件满足，进入步骤二
        if (timeInState >= SEQUENCE_DELAY_SEC) {
          setTargetState(HopperTargetState.DOWN_STOW);
        }
        break;

      case DOWN_STOW:
        // POV DOWN 终态：通道1(idx 0)设为0.3
        setServoPosition(0, 0.3);
        setServoPosition(1, 0.45);
        setServoPosition(2, 0.4);
        setServoPosition(5, 0.45);
        break;

        // ================= POV UP (指定模式) =================
      case UP_DEPLOY_STEP1:
        // POV UP 步骤一：通道1(idx 0)设为0
        setServoPosition(1, 0.0);
        setServoPosition(5, 0.00);

        // 延时条件满足，进入步骤二
        if (timeInState >= SEQUENCE_DELAY_SEC) {
          setTargetState(HopperTargetState.UP_DEPLOY);
        }
        break;
      case UP_DEPLOY:
        // POV UP 终态：通道3(idx 2)设为0.01
        setServoPosition(0, 0.0);
        setServoPosition(1, 0.0);
        setServoPosition(2, 0.01);
        setServoPosition(5, 0.00);
        break;

      case MID_INTAKE:
        setServoPosition(0, 0.5);
        setServoPosition(1, 0.5);
        setServoPosition(2, 0.5);
        break;
    }
    Logger.recordOutput("Hopper/TargetState", TargetState.name());
  }

  // =========================================================================

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
