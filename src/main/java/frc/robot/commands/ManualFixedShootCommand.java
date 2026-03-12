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

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManualFixedShootCommandConstants;
import frc.robot.Constants.MegaTrackIterativeCommandConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.WantedState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Manual shooting command with fixed close/far hood and flywheel setpoints. */
public class ManualFixedShootCommand extends Command {
  private static final String LOG_PREFIX = "ManualFixedShoot";

  public enum ShotState {
    CLOSE,
    FAR
  }

  private static ShotState selectedShotState = ShotState.CLOSE;

  private final RobotContainer robot;
  private final ShotState shotState;

  private static final double FEEDER_RPS = MegaTrackIterativeCommandConstants.FEEDER_RPS;
  private static final double INDEXER_VOLTS = MegaTrackIterativeCommandConstants.INDEXER_VOLTS;

  private static final double ENTER_FLYWHEEL_TOL =
      MegaTrackIterativeCommandConstants.ENTER_FLYWHEEL_RPS_TOL;
  private static final double EXIT_FLYWHEEL_TOL =
      MegaTrackIterativeCommandConstants.EXIT_FLYWHEEL_RPS_TOL;
  private static final double ENTER_HOOD_TOL =
      MegaTrackIterativeCommandConstants.ENTER_HOOD_DEG_TOL;
  private static final double EXIT_HOOD_TOL = MegaTrackIterativeCommandConstants.EXIT_HOOD_DEG_TOL;

  private final LoggedTunableNumber closeshooter =
      new LoggedTunableNumber(
          "Shooter/Fixed/closerps", ManualFixedShootCommandConstants.CLOSE_SHOOTER_RPS);
  private final LoggedTunableNumber farshooter =
      new LoggedTunableNumber(
          "Shooter/Fixed/farrps", ManualFixedShootCommandConstants.FAR_SHOOTER_RPS);
  private final LoggedTunableNumber farhood =
      new LoggedTunableNumber(
          "Shooter/Fixed/farhood", ManualFixedShootCommandConstants.FAR_HOOD_DEG);
  private boolean shooting = false;

  public ManualFixedShootCommand(RobotContainer robot, ShotState m_shotState) {
    this.robot = robot;
    this.shotState = m_shotState;
    addRequirements(robot.shooter, robot.hood, robot.feeder, robot.indexer, robot.intake);
  }

  public static ShotState getSelectedShotState() {
    return selectedShotState;
  }

  @Override
  public void initialize() {
    shooting = false;
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
  }

  @Override
  public void execute() {
    double shooterRpsSetpoint =
        shotState == ShotState.FAR ? farshooter.getAsDouble() : closeshooter.getAsDouble();
    double hoodDegSetpoint =
        shotState == ShotState.FAR
            ? farhood.getAsDouble()
            : ManualFixedShootCommandConstants.CLOSE_HOOD_DEG;

    robot.shooter.setVelocity(shooterRpsSetpoint);
    robot.hood.setAngle(hoodDegSetpoint);

    double flywheelErr = shooterRpsSetpoint - robot.shooter.getFlywheelVelocityRps();
    double hoodErr = hoodDegSetpoint - robot.hood.getAngleDeg();

    if (!shooting) {
      if (Math.abs(flywheelErr) <= ENTER_FLYWHEEL_TOL && Math.abs(hoodErr) <= ENTER_HOOD_TOL) {
        shooting = true;
      }
    } else {
      if (Math.abs(flywheelErr) > EXIT_FLYWHEEL_TOL || Math.abs(hoodErr) > EXIT_HOOD_TOL) {
        shooting = false;
      }
    }

    if (shooting) {
      int totalShots = robot.shooter.getShots1() + robot.shooter.getShots2();
      robot.intake.setWantedState(WantedState.FLICK_BACK);
      robot.intake.setShotCount(totalShots);
      robot.feeder.setVelocity(FEEDER_RPS);
      robot.indexer.setVoltage(INDEXER_VOLTS);
      robot.indexer.setUpVoltage(ManualFixedShootCommandConstants.INDEXER_UP_VOLTS);
    } else {
      robot.intake.setWantedState(WantedState.UP_STOW_STOP);
      robot.feeder.stop();
      robot.indexer.stop();
    }

    Logger.recordOutput(LOG_PREFIX + "/ShotState", shotState.name());
    Logger.recordOutput(LOG_PREFIX + "/ShooterRpsSetpoint", shooterRpsSetpoint);
    Logger.recordOutput(LOG_PREFIX + "/HoodDegSetpoint", hoodDegSetpoint);
    Logger.recordOutput(LOG_PREFIX + "/FlywheelErrRps", flywheelErr);
    Logger.recordOutput(LOG_PREFIX + "/HoodErrDeg", hoodErr);
    Logger.recordOutput(LOG_PREFIX + "/Shooting", shooting);
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
