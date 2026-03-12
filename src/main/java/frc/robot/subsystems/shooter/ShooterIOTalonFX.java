// Copyright 2025
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

/** TalonFX implementation for dual shooter flywheels. */
public class ShooterIOTalonFX implements ShooterIO {
  // Shooter 1 (leader + follower)
  private final TalonFX flywheel1Leader;
  private final TalonFX flywheel1Follower;
  private final VelocityTorqueCurrentFOC flywheel1VelocityReq = new VelocityTorqueCurrentFOC(0.0);
  private final TorqueCurrentFOC flywheel1TorqueReq = new TorqueCurrentFOC(0.0);
  private final DutyCycleOut flywheel1DutyReq = new DutyCycleOut(0.0);
  private final Follower flywheel1FollowerReq;

  // Shooter 2: 两个电机都作为 flywheel1Leader 的 follower
  private final TalonFX flywheel2Motor1;
  private final TalonFX flywheel2Motor2;
  private final Follower flywheel2Motor1FollowerReq;
  private final Follower flywheel2Motor2FollowerReq;

  // Shooter 1 signals
  private final StatusSignal<AngularVelocity> leader1Velocity;
  private final StatusSignal<AngularVelocity> follower1Velocity;
  private final StatusSignal<Voltage> leader1AppliedVolts;
  private final StatusSignal<Voltage> follower1AppliedVolts;
  private final StatusSignal<Current> leader1Current;
  private final StatusSignal<Current> follower1Current;

  // Shooter 2 signals
  private final StatusSignal<AngularVelocity> motor2_1Velocity;
  private final StatusSignal<AngularVelocity> motor2_2Velocity;
  private final StatusSignal<Voltage> motor2_1AppliedVolts;
  private final StatusSignal<Voltage> motor2_2AppliedVolts;
  private final StatusSignal<Current> motor2_1Current;
  private final StatusSignal<Current> motor2_2Current;

  public ShooterIOTalonFX() {
    flywheel1Leader = new TalonFX(ShooterConstants.FLYWHEEL_1_LEADER_ID, "mainCAN");
    flywheel1Follower = new TalonFX(ShooterConstants.FLYWHEEL_1_FOLLOWER_ID, "mainCAN");
    flywheel2Motor1 = new TalonFX(ShooterConstants.FLYWHEEL_2_LEADER_ID, "mainCAN");
    flywheel2Motor2 = new TalonFX(ShooterConstants.FLYWHEEL_2_FOLLOWER_ID, "mainCAN");

    var flywheelCfg = new TalonFXConfiguration();
    flywheelCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelCfg.MotorOutput.Inverted =
        ShooterConstants.FLYWHEEL_LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    flywheelCfg.TorqueCurrent.PeakReverseTorqueCurrent = -100;
    flywheelCfg.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    flywheelCfg.CurrentLimits.SupplyCurrentLimitEnable =
        ShooterConstants.ENABLE_SUPPLY_CURRENT_LIMIT;
    flywheelCfg.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    flywheelCfg.CurrentLimits.SupplyCurrentLowerLimit =
        ShooterConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    flywheelCfg.CurrentLimits.SupplyCurrentLowerTime =
        ShooterConstants.SUPPLY_CURRENT_LOWER_TIME_SEC;
    flywheelCfg.CurrentLimits.StatorCurrentLimitEnable =
        ShooterConstants.ENABLE_STATOR_CURRENT_LIMIT;
    flywheelCfg.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT_AMPS;

    // Configure shooter 1 leader
    flywheelCfg.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.FLYWHEEL_1_KP)
            .withKI(ShooterConstants.FLYWHEEL_1_KI)
            .withKD(ShooterConstants.FLYWHEEL_1_KD)
            .withKV(ShooterConstants.FLYWHEEL_1_KV)
            .withKS(ShooterConstants.FLYWHEEL_1_KS);
    flywheel1Leader.getConfigurator().apply(flywheelCfg);

    // Configure shooter 1 follower
    var followerCfg1 = new TalonFXConfiguration();
    followerCfg1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg1.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    followerCfg1.Slot0 = flywheelCfg.Slot0;
    followerCfg1.CurrentLimits = flywheelCfg.CurrentLimits;
    flywheel1Follower.getConfigurator().apply(followerCfg1);
    flywheel1FollowerReq =
        new Follower(flywheel1Leader.getDeviceID(), ShooterConstants.FLYWHEEL_1_FOLLOWER_INVERTED);
    flywheel1Follower.setControl(flywheel1FollowerReq);

    // Configure shooter 2: 两个电机都跟随 flywheel1Leader
    var followerCfg2 = new TalonFXConfiguration();
    followerCfg2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg2.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    followerCfg2.CurrentLimits = flywheelCfg.CurrentLimits;
    flywheel2Motor1.getConfigurator().apply(followerCfg2);
    flywheel2Motor2.getConfigurator().apply(followerCfg2);

    flywheel2Motor1FollowerReq =
        new Follower(flywheel1Leader.getDeviceID(), ShooterConstants.FLYWHEEL_2_FOLLOWER_INVERTED);
    flywheel2Motor2FollowerReq =
        new Follower(flywheel1Leader.getDeviceID(), ShooterConstants.FLYWHEEL_2_FOLLOWER_INVERTED);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);

    // Acquire status signals
    leader1Velocity = flywheel1Leader.getVelocity();
    follower1Velocity = flywheel1Follower.getVelocity();
    leader1AppliedVolts = flywheel1Leader.getMotorVoltage();
    follower1AppliedVolts = flywheel1Follower.getMotorVoltage();
    leader1Current = flywheel1Leader.getStatorCurrent();
    follower1Current = flywheel1Follower.getStatorCurrent();

    motor2_1Velocity = flywheel2Motor1.getVelocity();
    motor2_2Velocity = flywheel2Motor2.getVelocity();
    motor2_1AppliedVolts = flywheel2Motor1.getMotorVoltage();
    motor2_2AppliedVolts = flywheel2Motor2.getMotorVoltage();
    motor2_1Current = flywheel2Motor1.getStatorCurrent();
    motor2_2Current = flywheel2Motor2.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leader1Velocity,
        follower1Velocity,
        leader1AppliedVolts,
        follower1AppliedVolts,
        leader1Current,
        follower1Current,
        motor2_1Velocity,
        motor2_2Velocity,
        motor2_1AppliedVolts,
        motor2_2AppliedVolts,
        motor2_1Current,
        motor2_2Current);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status1 =
        BaseStatusSignal.refreshAll(
            leader1Velocity,
            follower1Velocity,
            leader1AppliedVolts,
            follower1AppliedVolts,
            leader1Current,
            follower1Current);
    var status2 =
        BaseStatusSignal.refreshAll(
            motor2_1Velocity,
            motor2_2Velocity,
            motor2_1AppliedVolts,
            motor2_2AppliedVolts,
            motor2_1Current,
            motor2_2Current);

    inputs.flywheel1LeaderVelocityRotationPerSec = leader1Velocity.getValueAsDouble();
    inputs.flywheel1FollowerVelocityRotationPerSec = follower1Velocity.getValueAsDouble();
    inputs.flywheel1LeaderAppliedVolts = leader1AppliedVolts.getValueAsDouble();
    inputs.flywheel1FollowerAppliedVolts = follower1AppliedVolts.getValueAsDouble();
    inputs.flywheel1LeaderCurrentAmps = leader1Current.getValueAsDouble();
    inputs.flywheel1FollowerCurrentAmps = follower1Current.getValueAsDouble();

    // Shooter2 现在是 follower, 日志字段名保持不变以兼容
    inputs.flywheel2LeaderVelocityRotationPerSec = motor2_1Velocity.getValueAsDouble();
    inputs.flywheel2FollowerVelocityRotationPerSec = motor2_2Velocity.getValueAsDouble();
    inputs.flywheel2LeaderAppliedVolts = motor2_1AppliedVolts.getValueAsDouble();
    inputs.flywheel2FollowerAppliedVolts = motor2_2AppliedVolts.getValueAsDouble();
    inputs.flywheel2LeaderCurrentAmps = motor2_1Current.getValueAsDouble();
    inputs.flywheel2FollowerCurrentAmps = motor2_2Current.getValueAsDouble();

    inputs.connected = status1.isOK() && status2.isOK();
  }

  @Override
  public void setFlywheelVelocity(double rps) {
    flywheel1Leader.setControl(flywheel1VelocityReq.withVelocity(rps));
    // 必须每周期重发 follower 请求，否则 Phoenix 6 控制请求超时后 follower 会停转
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);
  }

  @Override
  public void setFlywheelVelocity(double rps, double accelRpsPerSec) {
    flywheel1Leader.setControl(
        flywheel1VelocityReq.withVelocity(rps).withAcceleration(accelRpsPerSec));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);
  }

  @Override
  public void setFlywheelVelocity(double rps, double accelRpsPerSec, double torqueCurrentAmps) {
    flywheel1Leader.setControl(
        flywheel1VelocityReq
            .withVelocity(rps)
            .withAcceleration(accelRpsPerSec)
            .withFeedForward(torqueCurrentAmps));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);
  }

  @Override
  public void setFlywheelTorqueCurrent(double amps) {
    flywheel1Leader.setControl(flywheel1TorqueReq.withOutput(amps));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);
  }

  @Override
  public void setFlywheelDutyCycle(double dutyCycle) {
    flywheel1Leader.setControl(flywheel1DutyReq.withOutput(dutyCycle));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Motor1.setControl(flywheel2Motor1FollowerReq);
    flywheel2Motor2.setControl(flywheel2Motor2FollowerReq);
  }

  @Override
  public void stop() {
    flywheel1Leader.stopMotor();
    flywheel1Follower.stopMotor();
    flywheel2Motor1.stopMotor();
    flywheel2Motor2.stopMotor();
  }
}
