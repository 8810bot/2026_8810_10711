package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum WantedState {
    /** 向下放 + 吸球 */
    DOWN_INTAKE,
    /** 向上收回 + 停止吸球 */
    UP_STOW_STOP,
    /** 射球时：根据射球数量"越收越回" */
    SHOT_LINKED_STOW,
    UP_DEBUG,
    /** 间歇性收放，用来把球往后拨 */
    FLICK_BACK,
    /** 电流检测归零：小电压驱动到下止点，堵转后重设位置 */
    HOMING
  }

  private WantedState wantedState = WantedState.UP_STOW_STOP;

  private double rollerVoltsSetpoint = 0.0;
  private double deployPosRotSetpoint = 0.0;

  // Shot-linked stow
  private int shotCount = 0;

  // Flick timing
  private boolean flickDownPhase = false;
  private double nextFlickToggleTs = 0.0;

  // Homing state
  private boolean homed = false;
  private double homingStartTs = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    applyWantedState();

    Logger.recordOutput("Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Intake/RollerVoltsSetpoint", rollerVoltsSetpoint);
    Logger.recordOutput("Intake/DeployPosRotSetpoint", deployPosRotSetpoint);
    Logger.recordOutput("Intake/Homed", homed);
  }

  public void setWantedState(WantedState state) {
    if (state != wantedState) {
      wantedState = state;
      // Reset flick when entering flick mode
      if (wantedState == WantedState.FLICK_BACK) {
        flickDownPhase = false;
        nextFlickToggleTs = Timer.getFPGATimestamp();
      }
      // Record homing start time
      if (wantedState == WantedState.HOMING) {
        homingStartTs = Timer.getFPGATimestamp();
      }
    }
  }

  /** Provide total shots fired to drive {@link WantedState#SHOT_LINKED_STOW}. */
  public void setShotCount(int shotCount) {
    this.shotCount = Math.max(0, shotCount);
  }

  /** Backwards-compatible helper: directly set roller voltage and keep current deploy state. */
  public void setVoltage(double volts) {
    rollerVoltsSetpoint = volts;
    io.setRollerVoltage(volts);
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  /** Returns true if the homing routine has completed at least once. */
  public boolean isHomed() {
    return homed;
  }

  private void applyWantedState() {
    switch (wantedState) {
      case DOWN_INTAKE -> {
        deployPosRotSetpoint = IntakeConstants.DEPLOY_POS_DOWN_ROT;
        rollerVoltsSetpoint = IntakeConstants.ROLLER_INTAKE_VOLTS;
      }
      case UP_STOW_STOP -> {
        double baseUp = IntakeConstants.DEPLOY_POS_UP_ROT;
        deployPosRotSetpoint = baseUp;
        rollerVoltsSetpoint = IntakeConstants.ROLLER_STOP_VOLTS;
      }
      case UP_DEBUG -> {
        double baseUp = IntakeConstants.DEPLOY_POS_DEBUG_ROT;
        deployPosRotSetpoint = baseUp;
        rollerVoltsSetpoint = IntakeConstants.ROLLER_STOP_VOLTS;
      }
      case SHOT_LINKED_STOW -> {
        double baseUp = IntakeConstants.DEPLOY_POS_UP_ROT;
        double extra =
            MathUtil.clamp(
                shotCount * IntakeConstants.SHOOT_STOW_EXTRA_PER_SHOT_ROT,
                IntakeConstants.SHOOT_STOW_EXTRA_MIN_ROT,
                IntakeConstants.SHOOT_STOW_EXTRA_MAX_ROT);
        deployPosRotSetpoint = baseUp + extra;
        rollerVoltsSetpoint = IntakeConstants.ROLLER_STOP_VOLTS;

        Logger.recordOutput("Intake/ShotLinkedStow/Shots", shotCount);
        Logger.recordOutput("Intake/ShotLinkedStow/ExtraRot", extra);
      }
      case FLICK_BACK -> {
        double now = Timer.getFPGATimestamp();
        if (now >= nextFlickToggleTs) {
          flickDownPhase = !flickDownPhase;
          nextFlickToggleTs =
              now + (flickDownPhase ? IntakeConstants.FLICK_ON_SEC : IntakeConstants.FLICK_OFF_SEC);
        }
        deployPosRotSetpoint =
            flickDownPhase ? IntakeConstants.FLIP_POS_UP : IntakeConstants.DEPLOY_POS_UP_ROT;
        rollerVoltsSetpoint = IntakeConstants.FLICK_ROLLER_VOLTS;
      }
      case HOMING -> {
        rollerVoltsSetpoint = IntakeConstants.ROLLER_STOP_VOLTS;
        io.setRollerVoltage(rollerVoltsSetpoint);

        double elapsed = Timer.getFPGATimestamp() - homingStartTs;
        Logger.recordOutput("Intake/Homing/Current", inputs.deployCurrentAmps);
        Logger.recordOutput("Intake/Homing/Velocity", inputs.deployVelocityRotPerSec);
        Logger.recordOutput("Intake/Homing/Elapsed", elapsed);

        // 等待稳定时间后再检测堵转，避免启动电流瞬间误判
        if (elapsed >= IntakeConstants.HOMING_SETTLE_SEC
            && inputs.deployCurrentAmps > IntakeConstants.HOMING_CURRENT_THRESHOLD
            && Math.abs(inputs.deployVelocityRotPerSec)
                < IntakeConstants.HOMING_VELOCITY_THRESHOLD) {
          // 到达下止点：停止电压，重设位置为机械止点 (-70°)
          io.setDeployVoltage(0.0);
          io.resetDeployPosition(IntakeConstants.HOMING_RESET_POS_ROT);
          homed = true;
          // 归零完成，自动切换到收起状态
          wantedState = WantedState.UP_STOW_STOP;
          Logger.recordOutput("Intake/Homing/Complete", true);
        } else {
          // 尚未堵转，继续驱动
          io.setDeployVoltage(IntakeConstants.HOMING_VOLTAGE);
        }
        return; // HOMING 状态自行控制电机，跳过下方统一输出
      }
    }

    io.setDeployPositionRot(deployPosRotSetpoint);
    io.setRollerVoltage(rollerVoltsSetpoint);
  }

  public void stop() {
    setWantedState(WantedState.UP_STOW_STOP);
  }
}
