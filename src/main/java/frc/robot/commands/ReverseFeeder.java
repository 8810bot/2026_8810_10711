package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FeederConstants.UnjamConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;

public class ReverseFeeder extends Command {

  private final Feeder feeder;
  private final Indexer indexer;

  public ReverseFeeder(Feeder feeder, Indexer indexer) {
    this.feeder = feeder;
    this.indexer = indexer;
    addRequirements(feeder, indexer);
  }

  @Override
  public void initialize() {

    feeder.setVoltage(UnjamConstants.UNJAM_FEEDER_VOLTS);
    indexer.setVoltage(UnjamConstants.UNJAM_INDEXER_VOLTS);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    indexer.stop();
  }
}
