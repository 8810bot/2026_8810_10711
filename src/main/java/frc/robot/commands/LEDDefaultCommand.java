// Copyright 2026
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Default LED command.
 *
 * <p>TODO: fill in full state machine + blink patterns. For now this is a minimal placeholder that
 * demonstrates wiring.
 */
public class LEDDefaultCommand extends Command {
  private final RobotContainer robot;

  public LEDDefaultCommand(RobotContainer robot) {
    this.robot = robot;
    addRequirements(robot.led);
  }

  @Override
  public void execute() {
    robot.led.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
