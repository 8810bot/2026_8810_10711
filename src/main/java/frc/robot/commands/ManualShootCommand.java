package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MegaTrackIterativeCommandConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.WantedState;
import org.littletonrobotics.junction.Logger;

/**
 * 手动发射命令：无自瞄，使用 NT4 可调的射手速度和 hood 角度。
 *
 * <p>包含完整进料逻辑（feeder + indexer + intake 联动）。 飞轮到速 + hood 到位后自动启动进料，否则仅预旋飞轮。
 *
 * <p>按住按键期间持续运行，松开后停止所有电机。
 */
public class ManualShootCommand extends Command {
  private static final String LOG_PREFIX = "ManualShoot";

  private final RobotContainer robot;

  // 从 NT4 读取的 setpoint (通过 RobotContainer 里的 LoggedTunableNumber)
  private final java.util.function.DoubleSupplier shooterRpsSupplier;
  private final java.util.function.DoubleSupplier hoodDegSupplier;
  private final java.util.function.DoubleSupplier indexerUpVoltsSupplier;

  // 进料参数 (与 MegaTrackIterativeCommand 共用常量)
  private static final double FEEDER_RPS = MegaTrackIterativeCommandConstants.FEEDER_RPS;
  private static final double INDEXER_VOLTS = MegaTrackIterativeCommandConstants.INDEXER_VOLTS;

  // 射球门控容差 (与 MegaTrackIterativeCommand 保持一致)
  private static final double ENTER_FLYWHEEL_TOL =
      MegaTrackIterativeCommandConstants.ENTER_FLYWHEEL_RPS_TOL;
  private static final double EXIT_FLYWHEEL_TOL =
      MegaTrackIterativeCommandConstants.EXIT_FLYWHEEL_RPS_TOL;
  private static final double ENTER_HOOD_TOL =
      MegaTrackIterativeCommandConstants.ENTER_HOOD_DEG_TOL;
  private static final double EXIT_HOOD_TOL = MegaTrackIterativeCommandConstants.EXIT_HOOD_DEG_TOL;

  private double start_time;

  private boolean shooting = false;

  /**
   * @param robot RobotContainer
   * @param shooterRpsSupplier 飞轮速度 setpoint (RPS)，推荐是 LoggedTunableNumber::get
   * @param hoodDegSupplier hood 角度 setpoint (度)，推荐是 LoggedTunableNumber::get
   * @param indexerUpVoltsSupplier IndexerUp 电压 (V)，推荐是 LoggedTunableNumber::get
   */
  public ManualShootCommand(
      RobotContainer robot,
      java.util.function.DoubleSupplier shooterRpsSupplier,
      java.util.function.DoubleSupplier hoodDegSupplier,
      java.util.function.DoubleSupplier indexerUpVoltsSupplier) {
    this.robot = robot;
    this.shooterRpsSupplier = shooterRpsSupplier;
    this.hoodDegSupplier = hoodDegSupplier;
    this.indexerUpVoltsSupplier = indexerUpVoltsSupplier;
    addRequirements(robot.shooter, robot.hood, robot.feeder, robot.indexer, robot.intake);
  }

  @Override
  public void initialize() {
    shooting = false;
    start_time = Timer.getFPGATimestamp();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
  }

  @Override
  public void execute() {
    double shooterRps = shooterRpsSupplier.getAsDouble();
    double hoodDeg = hoodDegSupplier.getAsDouble();

    // 命令飞轮和 hood
    robot.shooter.setVelocity(shooterRps);
    robot.hood.setAngle(hoodDeg);

    // 计算误差
    double flywheelErr = shooterRps - robot.shooter.getFlywheelVelocityRps();
    double hoodErr = hoodDeg - robot.hood.getAngleDeg();
    // 门控：进入/退出 shooting 状态 (滞回)
    if (!shooting) {
      if (Math.abs(flywheelErr) <= ENTER_FLYWHEEL_TOL && Math.abs(hoodErr) <= ENTER_HOOD_TOL
          || Timer.getFPGATimestamp() - start_time > 2.0) {
        shooting = true;
      }
    } else {
      if (Math.abs(flywheelErr) > EXIT_FLYWHEEL_TOL || Math.abs(hoodErr) > EXIT_HOOD_TOL) {
        shooting = false;
      }
    }

    // 进料逻辑
    if (shooting) {
      int totalShots = robot.shooter.getShots1() + robot.shooter.getShots2();
      robot.intake.setWantedState(WantedState.FLICK_BACK);
      robot.intake.setShotCount(totalShots);
      robot.feeder.setVelocity(FEEDER_RPS);
      robot.indexer.setVoltage(INDEXER_VOLTS);
      robot.indexer.setUpVoltage(indexerUpVoltsSupplier.getAsDouble());
    } else {
      robot.intake.setWantedState(WantedState.UP_STOW_STOP);
      robot.feeder.stop();
      robot.indexer.stop();
    }

    // 日志
    Logger.recordOutput(LOG_PREFIX + "/ShooterRpsSetpoint", shooterRps);
    Logger.recordOutput(LOG_PREFIX + "/HoodDegSetpoint", hoodDeg);
    Logger.recordOutput(LOG_PREFIX + "/FlywheelErrRps", flywheelErr);
    Logger.recordOutput(LOG_PREFIX + "/HoodErrDeg", hoodErr);
    Logger.recordOutput(LOG_PREFIX + "/Shooting", shooting);
    Logger.recordOutput(LOG_PREFIX + "/MeasuredShooterRps", robot.shooter.getFlywheelVelocityRps());
    Logger.recordOutput(LOG_PREFIX + "/MeasuredHoodDeg", robot.hood.getAngleDeg());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    robot.shooter.stop();
    robot.hood.stop();
    robot.feeder.stop();
    robot.indexer.stop();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
    robot.shooter.resetShotCounts();
  }
}
