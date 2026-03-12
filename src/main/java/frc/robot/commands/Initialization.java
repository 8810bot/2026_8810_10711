package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Initialization extends Command {
  private final RobotContainer robot;

  public Initialization(RobotContainer robot) {
    this.robot = robot;
  }

  @Override
  public void initialize() {
    // No initialization needed
  }

  @Override
  public void execute() {
    // No execution needed
  }

  @Override
  public boolean isFinished() {
    return true; // Immediately finish after initialization
  }
}
