package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FeederConstants.UnjamConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;

public class FeederUnjamCommand extends Command {
  private enum State {
    NORMAL,
    UNJAM
  }

  private final Feeder feeder;
  private final Indexer indexer;

  private Debouncer jamDebouncer;
  private State state = State.NORMAL;
  private double unjamStartSec = Double.NaN;

  public FeederUnjamCommand(Feeder feeder, Indexer indexer) {
    this.feeder = feeder;
    this.indexer = indexer;
    addRequirements(feeder, indexer);
  }

  @Override
  public void initialize() {
    state = State.NORMAL;
    unjamStartSec = Double.NaN;
    jamDebouncer = new Debouncer(UnjamConstants.JAM_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
  }

  @Override
  public void execute() {
    boolean jamDetected =
        jamDebouncer.calculate(feeder.getCurrentAmps() > UnjamConstants.JAM_CURRENT_THRESHOLD_AMPS);

    if (state == State.NORMAL) {
      if (jamDetected) {
        state = State.UNJAM;
        unjamStartSec = Timer.getFPGATimestamp();
      } else {
        runNormal();
      }
    }

    if (state == State.UNJAM) {
      runUnjam();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    indexer.stop();
  }

  private void runNormal() {
    feeder.setVelocity(UnjamConstants.NORMAL_FEEDER_RPS);
    indexer.setVoltage(UnjamConstants.NORMAL_INDEXER_VOLTS);
  }

  private void runUnjam() {
    feeder.setVelocity(UnjamConstants.UNJAM_FEEDER_RPS);
    indexer.setVoltage(UnjamConstants.UNJAM_INDEXER_VOLTS);

    double elapsed = Timer.getFPGATimestamp() - unjamStartSec;
    if (elapsed >= UnjamConstants.UNJAM_TIME_SEC) {
      state = State.NORMAL;
    }
  }
}
