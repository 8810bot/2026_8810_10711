package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants.FlywheelControlConstants;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Flywheel control state machine:
 *
 * <ul>
 *   <li>STARTUP: duty-cycle boost until near setpoint
 *   <li>HOLD: velocity control with torque-current feedforward
 *   <li>BALL: torque-current control during ball contact
 *   <li>RECOVERY: duty-cycle boost to recover speed
 * </ul>
 */
public class FlywheelControlCommand extends Command {
  private enum State {
    STARTUP,
    HOLD,
    BALL,
    RECOVERY
  }

  private final Shooter shooter;
  private final DoubleSupplier targetRpsSupplier;
  private final BooleanSupplier ballPresentSupplier;

  private State state = State.STARTUP;
  private double stateStartSec = 0.0;

  public FlywheelControlCommand(
      Shooter shooter, DoubleSupplier targetRpsSupplier, BooleanSupplier ballPresentSupplier) {
    this.shooter = shooter;
    this.targetRpsSupplier = targetRpsSupplier;
    this.ballPresentSupplier = ballPresentSupplier;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    enterState(State.STARTUP);
  }

  @Override
  public void execute() {
    double targetRps = targetRpsSupplier.getAsDouble();
    shooter.setFlywheelSetpointRps(targetRps); // Step 0: keep setpoint updated for logging/arming.

    if (targetRps <= 0.0) {
      shooter.stop(); // Step 0: stop when no setpoint is requested.
      enterState(State.STARTUP);
      return;
    }

    double currentRps = shooter.getFlywheelVelocityRps();
    boolean ballPresent = ballPresentSupplier.getAsBoolean();
    boolean canSwitch = stateElapsedSec() >= FlywheelControlConstants.MODE_MIN_DWELL_SEC;

    switch (state) {
      case STARTUP -> {
        // Step 1: STARTUP boost to reach setpoint quickly.
        shooter.setDutyCycle(FlywheelControlConstants.BOOST_DUTY_CYCLE);
        boolean atSpeed =
            Math.abs(targetRps - currentRps)
                <= FlywheelControlConstants.STARTUP_ENTER_HOLD_BAND_RPS;
        if (atSpeed && canSwitch) {
          enterState(State.HOLD); // Step 1: enter HOLD once near setpoint.
        }
      }
      case HOLD -> {
        // Step 2: HOLD using velocity control + torque-current feedforward.
        shooter.setVelocity(
            targetRps,
            0.0,
            FlywheelControlConstants.HOLD_TORQUE_FF_AMPS);
        if (ballPresent && canSwitch) {
          enterState(State.BALL); // Step 2: switch to BALL when contact is detected.
        } else if (currentRps < targetRps - FlywheelControlConstants.RECOVERY_DROP_BAND_RPS
            && canSwitch) {
          enterState(State.RECOVERY); // Step 2: switch to RECOVERY if speed drops.
        }
      }
      case BALL -> {
        // Step 3: BALL phase with torque-current control for consistent torque.
        shooter.setTorqueCurrent(FlywheelControlConstants.BALL_TORQUE_CURRENT_AMPS);
        boolean timedOut =
            FlywheelControlConstants.BALL_PHASE_TIMEOUT_SEC > 0.0
                && stateElapsedSec()
                    >= FlywheelControlConstants.BALL_PHASE_TIMEOUT_SEC;
        if ((!ballPresent || timedOut) && canSwitch) {
          enterState(State.RECOVERY); // Step 3: exit BALL when contact ends or times out.
        }
      }
      case RECOVERY -> {
        // Step 4: RECOVERY boost to regain speed after ball exit.
        shooter.setDutyCycle(FlywheelControlConstants.BOOST_DUTY_CYCLE);
        boolean recovered =
            currentRps
                > targetRps - FlywheelControlConstants.RECOVERY_EXIT_BAND_RPS;
        if (recovered && canSwitch) {
          enterState(State.HOLD); // Step 4: return to HOLD once speed recovers.
        }
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  private void enterState(State next) {
    state = next;
    stateStartSec = Timer.getFPGATimestamp();
  }

  private double stateElapsedSec() {
    return Timer.getFPGATimestamp() - stateStartSec;
  }
}
