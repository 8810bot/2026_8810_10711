package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.MegaTrackVelocityProfileCommandConstants;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake.WantedState;
import frc.robot.util.MegaTrackVelocityProfileUtil;
import org.littletonrobotics.junction.Logger;

/** MegaTrack with iterative flight-time compensation and profiled rotational velocity command. */
public class MegaTrackVelocityProfileCommand extends Command {
  private enum State {
    ALIGN,
    SHOOT
  }

  private final RobotContainer robot;
  private final frc.robot.subsystems.drive.Drive drive;
  private final boolean isLob;
  private final java.util.function.DoubleSupplier xSupplier;
  private final java.util.function.DoubleSupplier ySupplier;

  private final PIDController headingController =
      new PIDController(
          MegaTrackVelocityProfileCommandConstants.HEADING_KP,
          MegaTrackVelocityProfileCommandConstants.HEADING_KI,
          MegaTrackVelocityProfileCommandConstants.HEADING_KD);

  private final TrapezoidProfile.Constraints omegaConstraints =
      new TrapezoidProfile.Constraints(
          MegaTrackVelocityProfileCommandConstants.OMEGA_MAX_RAD_PER_SEC,
          MegaTrackVelocityProfileCommandConstants.OMEGA_MAX_ACCEL_RAD_PER_SEC2);

  private Translation2d targetTranslation = new Translation2d();
  private Rotation2d heldHeading = new Rotation2d();
  private State state = State.ALIGN;
  private double lastFlightTimeSec = AutoShootConstants.FlyTime;

  private Translation2d shotOrigin = new Translation2d();
  private ChassisSpeeds vField = new ChassisSpeeds();
  private Translation2d aField = new Translation2d();
  private Translation2d compensatedVector = new Translation2d();
  private double dist = 0.0;
  private double tSec = AutoShootConstants.FlyTime;
  private double shooterRps = 0.0;
  private double hoodDeg = 0.0;

  private double omegaPid = 0.0;
  private double omegaFf = 0.0;
  private TrapezoidProfile.State omegaProfileState = new TrapezoidProfile.State(0.0, 0.0);

  private boolean distOk = false;
  private boolean triggerHeld = false;
  private double flywheelErrRps = 0.0;
  private double hoodErrDeg = 0.0;
  private double headingErrRad = 0.0;

  private double lastTimestampSec = 0.0;

  public MegaTrackVelocityProfileCommand(RobotContainer robot, boolean isLob) {
    this.robot = robot;
    this.drive = robot.drive;
    this.isLob = isLob;
    this.xSupplier = robot.getDriveXSupplier();
    this.ySupplier = robot.getDriveYSupplier();

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(
        robot.drive, robot.shooter, robot.hood, robot.feeder, robot.indexer, robot.intake);
  }

  private Translation2d getShotOriginField() {
    Pose2d pose = drive.getPose();
    Translation2d offsetRobot =
        new Translation2d(
            ShooterConstants.FLYWHEEL_OFFSET_X_METERS, ShooterConstants.FLYWHEEL_OFFSET_Y_METERS);
    return pose.getTranslation().plus(offsetRobot.rotateBy(pose.getRotation()));
  }

  @Override
  public void initialize() {
    headingController.reset();
    heldHeading = drive.getRotation();
    state = State.ALIGN;
    lastFlightTimeSec = AutoShootConstants.FlyTime;
    omegaProfileState = new TrapezoidProfile.State(0.0, 0.0);
    lastTimestampSec = Timer.getFPGATimestamp();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    if (isLob) {
      targetTranslation =
          FieldConstants.getNearestLobTarget(drive.getPose().getTranslation(), alliance);
    } else {
      targetTranslation = FieldConstants.getHubLocation(alliance);
    }

    Logger.recordOutput(MegaTrackVelocityProfileCommandConstants.LOG_PREFIX + "/IsLob", isLob);
    Logger.recordOutput(
        MegaTrackVelocityProfileCommandConstants.LOG_PREFIX + "/TargetTranslation",
        targetTranslation);
  }

  private void traceTargetAndControl() {
    shotOrigin = getShotOriginField();
    vField = drive.getFieldRelativeSpeeds();
    aField = drive.getFieldRelativeAcceleration();

    tSec =
        MegaTrackVelocityProfileUtil.iterateFlightTimeSec(
            targetTranslation,
            shotOrigin,
            vField,
            AutoShootConstants.flightTimeMap,
            lastFlightTimeSec,
            MegaTrackVelocityProfileCommandConstants.MAX_ITERS,
            MegaTrackVelocityProfileCommandConstants.EPS_T_SEC,
            MegaTrackVelocityProfileCommandConstants.RELAX,
            MegaTrackVelocityProfileCommandConstants.MIN_T_SEC,
            MegaTrackVelocityProfileCommandConstants.MAX_T_SEC);
    lastFlightTimeSec = tSec;

    compensatedVector =
        MegaTrackVelocityProfileUtil.compensatedVector(targetTranslation, shotOrigin, vField, tSec);
    dist = compensatedVector.getNorm();
    if (dist > 1e-3) {
      heldHeading = compensatedVector.getAngle();
    }

    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    omegaPid =
        headingController.calculate(drive.getRotation().getRadians(), heldHeading.getRadians());
    omegaFf =
        MegaTrackVelocityProfileUtil.omegaFeedforwardRadPerSec(
            compensatedVector,
            vField,
            aField,
            tSec,
            MegaTrackVelocityProfileCommandConstants.USE_ACCEL_IN_OMEGA_FF,
            MegaTrackVelocityProfileCommandConstants.MIN_OMEGA_FF_DIST_METERS,
            MegaTrackVelocityProfileCommandConstants.OMEGA_MAX_RAD_PER_SEC);

    double desiredOmega = omegaPid + omegaFf;
    desiredOmega =
        MathUtil.clamp(
            desiredOmega,
            -MegaTrackVelocityProfileCommandConstants.OMEGA_MAX_RAD_PER_SEC,
            MegaTrackVelocityProfileCommandConstants.OMEGA_MAX_RAD_PER_SEC);

    double nowSec = Timer.getFPGATimestamp();
    double dtSec = nowSec - lastTimestampSec;
    lastTimestampSec = nowSec;
    omegaProfileState =
        MegaTrackVelocityProfileUtil.stepOmegaProfile(
            omegaProfileState, desiredOmega, omegaConstraints, dtSec);

    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            linearVelocity.getX() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            linearVelocity.getY() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            omegaProfileState.position);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    shooterRps = AutoShootConstants.shooterSpeedMap.get(dist);
    hoodDeg = AutoShootConstants.hoodAngleMap.get(dist);
    robot.shooter.setVelocity(shooterRps);
    robot.hood.setAngle(hoodDeg);

    distOk =
        dist >= MegaTrackVelocityProfileCommandConstants.MIN_SHOOT_DIST_METERS
            && dist <= MegaTrackVelocityProfileCommandConstants.MAX_SHOOT_DIST_METERS;
    triggerHeld =
        robot.getRightTriggerAxisSupplier().getAsDouble()
            > MegaTrackVelocityProfileCommandConstants.TRIGGER_AXIS_THRESHOLD;
    flywheelErrRps = shooterRps - robot.shooter.getFlywheelVelocityRps();
    hoodErrDeg = hoodDeg - robot.hood.getAngleDeg();
    headingErrRad =
        MathUtil.angleModulus(heldHeading.getRadians() - drive.getRotation().getRadians());

    String lp = MegaTrackVelocityProfileCommandConstants.LOG_PREFIX;
    Logger.recordOutput(lp + "/State", state.toString());
    Logger.recordOutput(lp + "/ShotOrigin", shotOrigin);
    Logger.recordOutput(lp + "/T", tSec);
    Logger.recordOutput(lp + "/TargetPosition", compensatedVector.plus(shotOrigin));
    Logger.recordOutput(lp + "/Dist", dist);
    Logger.recordOutput(lp + "/HeldHeadingDeg", heldHeading.getDegrees());
    Logger.recordOutput(lp + "/OmegaPid", omegaPid);
    Logger.recordOutput(lp + "/OmegaFf", omegaFf);
    Logger.recordOutput(lp + "/OmegaDesired", desiredOmega);
    Logger.recordOutput(lp + "/OmegaProfiled", omegaProfileState.position);
    Logger.recordOutput(lp + "/TriggerHeld", triggerHeld);
    Logger.recordOutput(lp + "/FlywheelErrRps", flywheelErrRps);
    Logger.recordOutput(lp + "/HoodErrDeg", hoodErrDeg);
    Logger.recordOutput(lp + "/HeadingErrDeg", Math.toDegrees(headingErrRad));
  }

  private void stopFeed() {
    robot.feeder.stop();
    robot.indexer.stop();
  }

  private void runFeed() {
    robot.feeder.setVelocity(MegaTrackVelocityProfileCommandConstants.FEEDER_RPS);
    robot.indexer.setVoltage(MegaTrackVelocityProfileCommandConstants.INDEXER_VOLTS);
  }

  private boolean okToEnterShoot() {
    boolean ok =
        distOk
            && triggerHeld
            && Math.abs(flywheelErrRps)
                <= MegaTrackVelocityProfileCommandConstants.ENTER_FLYWHEEL_RPS_TOL
            && Math.abs(hoodErrDeg) <= MegaTrackVelocityProfileCommandConstants.ENTER_HOOD_DEG_TOL
            && Math.abs(headingErrRad)
                <= MegaTrackVelocityProfileCommandConstants.ENTER_HEADING_TOL_RAD;
    Logger.recordOutput(MegaTrackVelocityProfileCommandConstants.LOG_PREFIX + "/OkEnterShoot", ok);
    return ok;
  }

  private boolean okToStayShoot() {
    boolean ok =
        distOk
            && triggerHeld
            && Math.abs(flywheelErrRps)
                <= MegaTrackVelocityProfileCommandConstants.EXIT_FLYWHEEL_RPS_TOL
            && Math.abs(hoodErrDeg) <= MegaTrackVelocityProfileCommandConstants.EXIT_HOOD_DEG_TOL
            && Math.abs(headingErrRad)
                <= MegaTrackVelocityProfileCommandConstants.EXIT_HEADING_TOL_RAD;
    Logger.recordOutput(MegaTrackVelocityProfileCommandConstants.LOG_PREFIX + "/OkStayShoot", ok);
    return ok;
  }

  private void align() {
    traceTargetAndControl();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
    stopFeed();
    if (okToEnterShoot()) {
      state = State.SHOOT;
    }
  }

  private void shoot() {
    traceTargetAndControl();
    if (!okToStayShoot()) {
      stopFeed();
      state = State.ALIGN;
      return;
    }
    int totalShots = robot.shooter.getShots1() + robot.shooter.getShots2();
    robot.intake.setWantedState(WantedState.FLICK_BACK);
    robot.intake.setShotCount(totalShots);
    runFeed();
  }

  @Override
  public void execute() {
    switch (state) {
      case ALIGN -> align();
      case SHOOT -> shoot();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    robot.feeder.stop();
    robot.indexer.stop();
    robot.hood.stop();
    robot.shooter.stop();
    robot.shooter.resetShotCounts();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
  }
}
