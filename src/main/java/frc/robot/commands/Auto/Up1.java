package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.MegaTrackIterativeCommand;
import frc.robot.commands.ReverseFeeder;
import frc.robot.commands.SmashTrenchCommand;
import frc.robot.subsystems.hopper.Hopper.HopperTargetState;
import frc.robot.subsystems.intake.Intake;

public class Up1 extends SequentialCommandGroup {

  public Up1(RobotContainer robotContainer) {
    PathPlannerPath up1Path;
    try {
      up1Path = PathPlannerPath.fromChoreoTrajectory("Up1");

      addCommands(
          new InstantCommand(() -> robotContainer.intake.setWantedState(Intake.WantedState.INIT)));

      addCommands(
          new ReverseFeeder(robotContainer.feeder, robotContainer.indexer)
              .raceWith(new WaitCommand(0.5)));
      addCommands(
          new InstantCommand(
              () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      addCommands(
          new InstantCommand(
              () -> robotContainer.hopper.setTargetState(HopperTargetState.DOWN_STOW_STEP1)));
      addCommands(new WaitCommand(0.5));
      addCommands(new SmashTrenchCommand(robotContainer).withTimeout(3.2));
      addCommands(
          new InstantCommand(
              () -> robotContainer.hopper.setTargetState(HopperTargetState.UP_DEPLOY_STEP1)));
      addCommands(AutoBuilder.followPath(up1Path));
      addCommands(new MegaTrackIterativeCommand(robotContainer, false).withTimeout(10));
      // addCommands(new SmashTrenchCommand(robotContainer).withTimeout(3.2));
      // addCommands(
      //     new InstantCommand(
      //         () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      // addCommands(AutoBuilder.followPath(Down2));
      // addCommands(new SmashBumpCommand(robotContainer).withTimeout(3.2));
      // addCommands(new MegaTrackIterativeCommand(robotContainer, false));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public static Pose2d getStartPose(Alliance alliance) {
    Pose2d bluePose2d = new Pose2d(4.59, 0.51, new Rotation2d(Degrees.of(0)));
    if (alliance == Alliance.Blue) return bluePose2d;
    else return bluePose2d.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.k180deg);
  }
}
