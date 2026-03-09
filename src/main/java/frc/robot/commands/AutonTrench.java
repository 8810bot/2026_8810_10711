package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AutonTrenchUtil;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AutonTrench extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final PIDController aimPidController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController yPidController = new PIDController(5.0, 0, 0);
  private double yTarget;
  private double rotTarget;

  public AutonTrench(Drive mDrive, DoubleSupplier iXSupplier) {
    drive = mDrive;
    addRequirements(drive);
    xSupplier = iXSupplier;
    yPidController.setTolerance(0.1);
    aimPidController.enableContinuousInput(-Math.PI, Math.PI);
    aimPidController.setTolerance(Math.toRadians(5));
  }

  @Override
  public void initialize() {
    Pose2d pose = drive.getPose();
    yTarget = AutonTrenchUtil.selectYTarget(pose.getY());
    rotTarget = AutonTrenchUtil.selectRotationTarget(pose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getPose();
    double robotY = pose.getY();
    double robotYaw = pose.getRotation().getRadians();

    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), 0);

    double rotSpeed = aimPidController.calculate(robotYaw, rotTarget);
    double ySpeed = yPidController.calculate(robotY, yTarget);

    ySpeed = yPidController.atSetpoint() ? 0 : ySpeed;
    rotSpeed = aimPidController.atSetpoint() ? 0 : rotSpeed;

    ChassisSpeeds aimSpeed =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            AutonTrenchUtil.clampYSpeed(ySpeed),
            rotSpeed);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(aimSpeed, pose.getRotation()));

    Logger.recordOutput("AutoTrench/IsYAtSetpoint", yPidController.atSetpoint());
    Logger.recordOutput("AutoTrench/IsYawAtSetpoint", aimPidController.atSetpoint());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
