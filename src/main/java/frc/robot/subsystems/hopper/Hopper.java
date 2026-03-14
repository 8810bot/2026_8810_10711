package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper subsystem, including piece detection and servo interaction helpers.
 *
 * <p>Uses two CANrange sensors and a Debouncer to determine whether the hopper is "full".
 */
public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  // --- 安全互锁: Pose 供应器 ---
  private final Supplier<Pose2d> poseSupplier;

  // 进入危险区的迟滞 (Hysteresis) 状态
  private boolean trenchSafetyActive = false;

  // 手动覆盖标志：持续置 true 时，safety interlock 暂停执行
  private boolean safetyOverrideActive = false;

  public enum HopperTargetState {
    DOWN_STOW_STEP1,
    DOWN_STOW_STEP2,
    DOWN_STOW,
    UP_DEPLOY_STEP1,
    UP_DEPLOY_STEP2,
    UP_DEPLOY,
    MID_INTAKE
  }

  private HopperTargetState TargetState = HopperTargetState.DOWN_STOW;
  private double stateStartTime = 0.0;
  private static final double SEQUENCE_DELAY_SEC = 0.2;

  private final Debouncer fullDebouncer =
      new Debouncer(HopperConstants.FULL_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

  private boolean full = false;

  public Hopper(HopperIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    init();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // --- 全局安全互锁: 分析当前姿态，判断是否在战壕危险区 ---
    Pose2d currentPose = poseSupplier.get();
    double rx = currentPose.getX();
    double ry = currentPose.getY();
    boolean nowInDanger = false;
    // 已在区内：扩大边界（exitExpand 向外）才能退出，防止颤振
    // 未在区内：缩小边界（enterShrink 向内）才能进入，需要更深才触发
    double enterShrink = HopperConstants.TRENCH_HYSTERESIS_ENTER_MARGIN;
    double exitExpand = HopperConstants.TRENCH_HYSTERESIS_EXIT_MARGIN;
    for (double[] zone : HopperConstants.TRENCH_DANGER_ZONES) {
      double xMin =
          trenchSafetyActive ? zone[0] - exitExpand : zone[0] + enterShrink;
      double xMax =
          trenchSafetyActive ? zone[1] + exitExpand : zone[1] - enterShrink;
      double yMin =
          trenchSafetyActive ? zone[2] - exitExpand : zone[2] + enterShrink;
      double yMax =
          trenchSafetyActive ? zone[3] + exitExpand : zone[3] - enterShrink;
      if (rx >= xMin && rx <= xMax && ry >= yMin && ry <= yMax) {
        nowInDanger = true;
        break;
      }
    }
    trenchSafetyActive = nowInDanger;

    // 若处于危险区且当前状态不是收拢系列，且未手动覆盖，则强制收拢
    if (trenchSafetyActive && !safetyOverrideActive && isExpandedState(TargetState)) {
      TargetState = HopperTargetState.DOWN_STOW_STEP1;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

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
    Logger.recordOutput("Hopper/TrenchSafetyActive", trenchSafetyActive);
    Logger.recordOutput("Hopper/SafetyOverrideActive", safetyOverrideActive);
  }

  public void setTargetState(HopperTargetState state) {
    // 安全门禁：在战壕危险区内且未覆盖，拒绝切换到展开状态
    if (trenchSafetyActive && !safetyOverrideActive && isExpandedState(state)) {
      Logger.recordOutput("Hopper/SafetyBlocked", true);
      return;
    }
    Logger.recordOutput("Hopper/SafetyBlocked", false);
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

  /** True if hopper is currently in a trench danger zone (safety interlock active). */
  public boolean isTrenchSafetyActive() {
    return trenchSafetyActive;
  }

  /**
   * 设置手动安全覆盖。
   *
   * <p>置 true 时，战壕安全互锁暂停：periodic() 不强制收拢，setTargetState() 不拦截展开请求。
   * 按键松开后应立即调用 setSafetyOverride(false) 恢复保护。
   */
  public void setSafetyOverride(boolean override) {
    safetyOverrideActive = override;
    Logger.recordOutput("Hopper/SafetyOverrideActive", override);
  }

  /** Returns whether the given state is one of the expanded / deploy states. */
  private boolean isExpandedState(HopperTargetState state) {
    return state == HopperTargetState.UP_DEPLOY
        || state == HopperTargetState.UP_DEPLOY_STEP1
        || state == HopperTargetState.UP_DEPLOY_STEP2;
  }

  /** Set one servo position by index [0..4]. */
  public void setServoPosition(int index, double position) {
    io.setServoPosition(index, position);
    Logger.recordOutput("Hopper/Servo" + (index) + "Setpoint", position);
  }

  private void init() {
    setServoPosition(0, 0.3);
    setServoPosition(1, 0.4);
    setServoPosition(2, 0.45);
    setServoPosition(5, 0.45);
  }
  /** Apply the target state to the hopper servos. */
  private void applyTargetState() {
    double timeInState = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;

    switch (TargetState) {
        // ================= POV DOWN (全变 0.5) =================
      case DOWN_STOW_STEP1:
        // POV DOWN 步骤一：相反顺序，先设置通道3(idx 2)为0.35
        setServoPosition(1, 0.4);
        setServoPosition(0, 0.45);

        // 延时条件满足，进入步骤二
        if (timeInState >= SEQUENCE_DELAY_SEC) {
          setTargetState(HopperTargetState.DOWN_STOW);
        }
        break;

      case DOWN_STOW_STEP2:
        // 目前未使用此中间过渡状态，直接迬就到终态
        setTargetState(HopperTargetState.DOWN_STOW);
        break;

      case DOWN_STOW:
        // POV DOWN 终态：通道1(idx 0)设为0.3
        setServoPosition(0, 0.3);
        setServoPosition(2, 0.45);
        setServoPosition(1, 0.4);
        setServoPosition(5, 0.45);
        break;

        // ================= POV UP (指定模式) =================
      case UP_DEPLOY_STEP1:
        // POV UP 步骤一：通道1(idx 0)设为0
        setServoPosition(2, 0.0);
        setServoPosition(5, 0.00);

        // 延时条件满足，进入步骤二
        if (timeInState >= SEQUENCE_DELAY_SEC) {
          setTargetState(HopperTargetState.UP_DEPLOY);
        }
        break;

      case UP_DEPLOY_STEP2:
        // 目前未使用此中间过渡状态，直接跳到终态
        setTargetState(HopperTargetState.UP_DEPLOY);
        break;
      case UP_DEPLOY:
        // POV UP 终态：通道3(idx 2)设为0.01
        setServoPosition(0, 0.0);
        setServoPosition(2, 0.0);
        setServoPosition(1, 0.01);
        setServoPosition(5, 0.00);
        break;

      case MID_INTAKE:
        setServoPosition(0, 0.5);
        setServoPosition(2, 0.5);
        setServoPosition(1, 0.5);
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
