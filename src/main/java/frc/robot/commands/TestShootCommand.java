package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake.WantedState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Test shooting command for calibrating interpolation tables.
 *
 * <p>Mode is controlled by a NT4 tunable parameter (useInterpolationSupplier):
 *
 * <ul>
 *   <li><b>0 = Direct mode</b>: manually specify shooter RPS and hood angle via NT4, used to find
 *       the best parameters at a given distance, then fill into interpolation tables.
 *   <li><b>1 = Interpolation mode</b>: uses the real-time distance to hub to query shooterSpeedMap
 *       and hoodAngleMap (same as MegaTrackIterativeCommand), used to verify the tables are
 *       correct.
 * </ul>
 *
 * <p>Mode can be switched live via dashboard without releasing the button.
 *
 * <p>Both modes log distance-to-hub, setpoints, measured values, table lookups, and errors via NT4
 * for calibration.
 *
 * <p>Right trigger gates feeding (safety).
 */
public class TestShootCommand extends Command {
  private final RobotContainer robotContainer;
  private final DoubleSupplier useInterpolationSupplier;
  private final DoubleSupplier shooterRpsSupplier;
  private final DoubleSupplier hoodDegSupplier;
  private final DoubleSupplier feederRpsSupplier;
  private final DoubleSupplier indexerVoltsSupplier;
  private final double triggerThreshold;

  /**
   * @param robotContainer RobotContainer
   * @param useInterpolationSupplier 0 = direct mode, nonzero = interpolation mode, NT4 tunable
   * @param shooterRpsSupplier flywheel velocity setpoint (RPS), used in direct mode, NT4 tunable
   * @param hoodDegSupplier hood angle setpoint (degrees), used in direct mode, NT4 tunable
   * @param feederRpsSupplier feeder velocity (RPS), NT4 tunable
   * @param indexerVoltsSupplier indexer voltage (V), NT4 tunable
   * @param triggerThreshold right trigger axis threshold to start feeding
   */
  public TestShootCommand(
      RobotContainer robotContainer,
      DoubleSupplier useInterpolationSupplier,
      DoubleSupplier shooterRpsSupplier,
      DoubleSupplier hoodDegSupplier,
      DoubleSupplier feederRpsSupplier,
      DoubleSupplier indexerVoltsSupplier,
      double triggerThreshold) {
    this.robotContainer = robotContainer;
    this.useInterpolationSupplier = useInterpolationSupplier;
    this.shooterRpsSupplier = shooterRpsSupplier;
    this.hoodDegSupplier = hoodDegSupplier;
    this.feederRpsSupplier = feederRpsSupplier;
    this.indexerVoltsSupplier = indexerVoltsSupplier;
    this.triggerThreshold = triggerThreshold;
    addRequirements(
        robotContainer.shooter,
        robotContainer.hood,
        robotContainer.feeder,
        robotContainer.indexer,
        robotContainer.intake);
  }

  private double getDistToHub() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation2d hubLocation = FieldConstants.getHubLocation(alliance);
    Translation2d robotTranslation = robotContainer.drive.getPose().getTranslation();
    Translation2d shotOrigin =
        robotTranslation.plus(
            new Translation2d(
                    ShooterConstants.FLYWHEEL_OFFSET_X_METERS,
                    ShooterConstants.FLYWHEEL_OFFSET_Y_METERS)
                .rotateBy(robotContainer.drive.getPose().getRotation()));
    return shotOrigin.getDistance(hubLocation);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double feederRps = feederRpsSupplier.getAsDouble();
    double indexerVolts = indexerVoltsSupplier.getAsDouble();
    double distToHub = getDistToHub();
    boolean useInterpolation = useInterpolationSupplier.getAsDouble() != 0.0;

    double shooterRps;
    double hoodDeg;
    if (useInterpolation) {
      shooterRps = AutoShootConstants.shooterSpeedMap.get(distToHub);
      hoodDeg = AutoShootConstants.hoodAngleMap.get(distToHub);
      Logger.recordOutput("TestShoot/Mode", "Interpolated");
    } else {
      shooterRps = shooterRpsSupplier.getAsDouble();
      hoodDeg = hoodDegSupplier.getAsDouble();
      Logger.recordOutput("TestShoot/Mode", "Direct");
    }

    // Measured values
    double measuredShooterRps = robotContainer.shooter.getFlywheelVelocityRps();
    double measuredHoodDeg = robotContainer.hood.getAngleDeg();

    // Command shooter and hood
    robotContainer.shooter.setVelocity(shooterRps);
    robotContainer.hood.setAngle(hoodDeg);

    // Log: setpoints
    Logger.recordOutput("TestShoot/ShooterRpsSetpoint", shooterRps);
    Logger.recordOutput("TestShoot/HoodDegSetpoint", hoodDeg);
    Logger.recordOutput("TestShoot/FeederRpsSetpoint", feederRps);
    Logger.recordOutput("TestShoot/IndexerVoltsSetpoint", indexerVolts);

    // Log: measurements for table calibration
    Logger.recordOutput("TestShoot/DistToHubMeters", distToHub);
    Logger.recordOutput("TestShoot/MeasuredShooterRps", measuredShooterRps);
    Logger.recordOutput("TestShoot/MeasuredHoodDeg", measuredHoodDeg);
    Logger.recordOutput("TestShoot/ShooterErrRps", shooterRps - measuredShooterRps);
    Logger.recordOutput("TestShoot/HoodErrDeg", hoodDeg - measuredHoodDeg);

    // Log: interpolation table lookup (always show for comparison, even in direct mode)
    Logger.recordOutput(
        "TestShoot/TableShooterRps", AutoShootConstants.shooterSpeedMap.get(distToHub));
    Logger.recordOutput("TestShoot/TableHoodDeg", AutoShootConstants.hoodAngleMap.get(distToHub));

    // Feed control (right trigger gated)
    if (robotContainer.getRightTriggerAxisSupplier().getAsDouble() > triggerThreshold) {
      int totalShots = robotContainer.shooter.getShots1() + robotContainer.shooter.getShots2();
      robotContainer.intake.setWantedState(WantedState.SHOT_LINKED_STOW);
      robotContainer.intake.setShotCount(totalShots);

      robotContainer.feeder.setVelocity(feederRps);
      robotContainer.indexer.setVoltage(indexerVolts);
    } else {
      robotContainer.intake.setWantedState(WantedState.UP_STOW_STOP);
      robotContainer.feeder.stop();
      robotContainer.indexer.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.feeder.stop();
    robotContainer.indexer.stop();
    robotContainer.intake.setWantedState(WantedState.UP_STOW_STOP);
    robotContainer.hood.stop();
    robotContainer.shooter.stop();
  }
}
