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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.servohub.ServoChannel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.Auto.Down;
import frc.robot.commands.Auto.DownMagic;
import frc.robot.commands.Auto.Up1;
import frc.robot.commands.Auto.UpOut;
import frc.robot.commands.AutonTrench;
import frc.robot.commands.DefaultFeederCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.MegaTrackIterativeCommand;
import frc.robot.commands.ReverseFeeder;
import frc.robot.commands.SmashBumpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  // Dashboard tuning
  private LoggedTunableNumber hoodAngleDegTunable = new LoggedTunableNumber("Hood/AngleDeg", 10);
  private LoggedTunableNumber shooterVelRpsTunable = new LoggedTunableNumber("Shooter/VelRps", 22);
  private LoggedTunableNumber enableAutoAimTunable =
      new LoggedTunableNumber("Shooter/EnableAutoAim", 1);

  // ---- IndexerUp 独立电压控制 ----
  /** 仪表盘实时调节 IndexerUp 电压 (volts) */
  private final LoggedTunableNumber indexerUpVolts =
      new LoggedTunableNumber("IndexerUp/Volts", 8.0);

  @SuppressWarnings("unused")
  public final Shooter shooter;

  @SuppressWarnings("unused")
  public final Hood hood;

  @SuppressWarnings("unused")
  public final Feeder feeder;

  @SuppressWarnings("unused")
  public final Intake intake;

  @SuppressWarnings("unused")
  public final Indexer indexer;

  @SuppressWarnings("unused")
  public final Hopper hopper;

  @SuppressWarnings("unused")
  public final LED led;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  /** Driver X speed supplier (forward/back). */
  public DoubleSupplier getDriveXSupplier() {
    return () -> -controller.getLeftY();
  }

  /** Driver Y speed supplier (left/right). */
  public DoubleSupplier getDriveYSupplier() {
    return () -> -controller.getLeftX();
  }

  /** Right trigger axis supplier in range [0, 1]. */
  public DoubleSupplier getRightTriggerAxisSupplier() {
    return () -> controller.getRightTriggerAxis();
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  ServoChannel m_channel5;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // if (true) {
    //   ServoHub m_servoHub = new ServoHub(3);
    //   ServoHubConfig config = new ServoHubConfig();
    //   config
    //       .channel5
    //       .pulseRange(500, 1500, 2500)
    //       .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

    //   // Persist parameters and reset any not explicitly set above to
    //   // their defaults.
    //   m_servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
    //   m_channel5 = m_servoHub.getServoChannel(ChannelId.kChannelId5);
    //   m_servoHub.setBankPulsePeriod(ServoHub.Bank.kBank3_5, 20000);
    //   m_channel5.setPowered(true);
    //   m_channel5.setEnabled(true);
    // }

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        hopper = new Hopper(new HopperIOReal(), drive::getPose);
        indexer = new Indexer(new IndexerIOTalonFX());
        // 暂时禁用真实 CANdle 消除报错: led = new LED(new LEDIOCANdle());
        led = new LED(new LEDIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hopper = new Hopper(new HopperIO() {}, drive::getPose);
        led = new LED(new LEDIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hopper = new Hopper(new HopperIO() {}, drive::getPose);
        led = new LED(new LEDIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption("Up", new UpOut(this).withTimeout(20.5));
    autoChooser.addOption("LEFT1", new Up1(this).withTimeout(20.5));
    autoChooser.addOption("Magic", new DownMagic(this).withTimeout(20.5));

    autoChooser.addOption("Down", new Down(this).withTimeout(20.5));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("test", new PathPlannerAuto("test"));
    autoChooser.addOption("CIRCLE", new PathPlannerAuto("CIRCLE"));
    autoChooser.addOption("straight", new PathPlannerAuto("straight"));

    // Configure the button bindings
    configureButtonBindings();

    // Default commands
    led.setDefaultCommand(new LEDDefaultCommand(this));
    feeder.setDefaultCommand(new DefaultFeederCommand(feeder));
    // indexer.setDefaultCommand(new DefaultIndexerCommand(indexer));
  }

  // IndexerUp：仪表盘实时调节电压
  //   new edu.wpi.first.wpilibj2.command.button.Trigger(() -> indexerUpVolts.hasChanged(0))
  //       .onTrue(
  //           Commands.runOnce(
  //               () -> {
  //                 double volts = indexerUpVolts.get();
  //                 indexer.setUpVoltage(volts);
  //               }));

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // controller
    //     .povUp()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               drive.runVelocity(new ChassisSpeeds(2, 0, 0));
    //             },
    //             drive));

    // Debug: log nearest trench pre-align pose

    // 自动对准Trench，右摇杆按住时进入，松开时退出
    controller.rightStick().whileTrue(new AutonTrench(drive, () -> controller.getLeftY()));
    controller
        .rightStick()
        .onTrue(
            new InstantCommand(
                () -> hopper.setTargetState(Hopper.HopperTargetState.DOWN_STOW_STEP1)));
    controller
        .rightStick()
        .onFalse(
            new InstantCommand(
                () -> hopper.setTargetState(Hopper.HopperTargetState.UP_DEPLOY_STEP1)));
    controller.leftStick().whileTrue(new SmashBumpCommand(this));

    // Manual tuning buttons
    // controller
    //     .a()
    //     .whileTrue(new InstantCommand(() -> shooter.setVelocity(shooterVelRpsTunable.get())));
    // controller.x().whileTrue(new InstantCommand(() -> hood.setAngle(hoodAngleDegTunable.get())));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // LB 键：按下开吸球，松开停吸球 (保持在下面)
    controller
        .leftBumper()
        .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));

    controller.leftBumper().whileTrue(new InstantCommand(() -> indexer.setUpVoltage(-3), indexer));
    controller
        .leftBumper()
        .onFalse(
            new InstantCommand(() -> intake.setWantedState(Intake.WantedState.DOWN_IDLE))
                .alongWith(new InstantCommand(() -> indexer.setUpVoltage(0), indexer)));

    // X 键：按住收回 Intake 到抬升位置
    controller
        .x()
        .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.UP_STOW_STOP)));
    controller
        .x()
        .onFalse(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.DOWN_IDLE)));

    controller.a().whileTrue(new ReverseFeeder(feeder, indexer));
    // controller
    //     .povDown()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               Logger.recordOutput(
    //                   "Bump/AlignPose", FieldConstants.getNearestTrenchPrePose(drive.getPose()));
    //             }));

    // POV Down 已改用于控制 Hopper 伺服 (DOWN_STOW)
    // 如需手动调 Hood 角度，请通过 NT4 仪表盘直接修改后重新部署，或使用其他按键
    // controller
    //     .povDown()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               double deg = hoodAngleDegTunable.get();
    //               hood.setAngle(deg);
    //             },
    //             hood));

    // Reset gyro to 0° when B button is pressed
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        if (autoChooser.getSendableChooser().getSelected() == "LEFT1") {
                          drive.setPose(UpOut.getStartPose(DriverStation.getAlliance().get()));
                        } else {
                          drive.setPose(DownMagic.getStartPose(DriverStation.getAlliance().get()));
                        }
                      } else {
                        drive.setPose(
                            DriverStation.getAlliance().get() == Alliance.Blue
                                ? new Pose2d(3.22, 4.030, new Rotation2d())
                                : new Pose2d(3.22, 4.030, new Rotation2d())
                                    .rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.k180deg));
                      }
                    },
                    drive)
                .ignoringDisable(true));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> drive.setPose(UpOut.getStartPose(DriverStation.getAlliance().get())),
    //                 drive)
    //             .ignoringDisable(true));

    // if (m_channel5 != null) {
    // Y 键 → 专用安全覆盖锁：按住时暂停战壕互锁，松开后立即恢复保护
    controller
        .y()
        .whileTrue(
            Commands.startEnd(
                () -> hopper.setSafetyOverride(true),
                () -> hopper.setSafetyOverride(false),
                hopper));
    //   controller
    //       .b()
    //       .whileTrue(
    //           new InstantCommand(() -> this.m_channel5.setPulseWidth(1500))
    //               .alongWith(new InstantCommand(() -> this.m_channel5.setPowered(true)))
    //               .alongWith(new InstantCommand(() -> this.m_channel5.setEnabled(true))));
    // }
    // controller
    //     .a()
    //     .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.UP_DEBUG)));

    // controller
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               intake.setWantedState(Intake.WantedState.FLICK_BACK);
    //               feeder.setVelocity(leftBumperFeederRps.get());
    //               indexer.setVoltage(leftBumperIndexerVolts.get());
    //             },
    //             intake,
    //             feeder,
    //             indexer))
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               intake.setWantedState(Intake.WantedState.UP_STOW_STOP);
    //               feeder.stop();
    //               indexer.stop();
    //             },
    //             intake,
    //             feeder,
    //             indexer));
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         Commands.run(
    //             () -> {
    //               intake.setWantedState(Intake.WantedState.FLICK_BACK);
    //             },
    //             intake))
    //     .onFalse(
    //         Commands.runOnce(
    //             () -> {
    //               intake.setWantedState(Intake.WantedState.DOWN_IDLE);
    //             },
    //             intake));
    // 手动发射 (无自瞄，NT4 可调参数: Shooter/VelRps, Hood/AngleDeg)
    controller
        .b()
        .whileTrue(
            new ManualShootCommand(
                    this,
                    () -> shooterVelRpsTunable.get(),
                    () -> hoodAngleDegTunable.get(),
                    () -> indexerUpVolts.get())
                .andThen(new InstantCommand(() -> intake.setVoltage(7), intake)));

    // 射击指令 Autoaim 开关由仪表盘参数 Shooter/EnableAutoAim 控制，>0 时启用自动瞄准，否则手动瞄准
    controller
        .rightBumper()
        .whileTrue(
            Commands.either(
                new MegaTrackIterativeCommand(this, false),
                new ManualShootCommand(
                    this,
                    () -> shooterVelRpsTunable.get(),
                    () -> hoodAngleDegTunable.get(),
                    () -> indexerUpVolts.get()),
                () -> enableAutoAimTunable.get() > 0));

    // 强制 Manual射击
    controller
        .rightTrigger()
        .whileTrue(
            Commands.either(
                new MegaTrackIterativeCommand(this, true),
                new ManualShootCommand(
                    this,
                    () -> shooterVelRpsTunable.get(),
                    () -> hoodAngleDegTunable.get(),
                    () -> indexerUpVolts.get()),
                () -> enableAutoAimTunable.get() > 0));
    // POV Up → 触发内部状态机序列: 切换到 UP_DEPLOY_STEP1 进行延时
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> hopper.setTargetState(Hopper.HopperTargetState.UP_DEPLOY_STEP1), hopper));
    // POV Down → 触发内部状态机序列: 切换到 DOWN_STOW_STEP1 进行延时
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> hopper.setTargetState(Hopper.HopperTargetState.DOWN_STOW_STEP1), hopper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Called automatically when the robot is enabled (in Auto or Teleop). */
  public void onEnable() {
    // 自动将当前位置作为 Hood 的零点 (0度)
    // intake.setWantedState(Intake.WantedState.INIT);

    hood.zeroPosition();
  }
}
