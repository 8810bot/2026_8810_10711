package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX motor1;
  private final TalonFX motor2;
  private final TalonFX motorUp;

  private final VoltageOut voltageReq1 = new VoltageOut(0.0);
  private final VoltageOut voltageReq2 = new VoltageOut(0.0);
  private final VoltageOut voltageReqUp = new VoltageOut(0.0);

  private final StatusSignal<Voltage> appliedVolts1;
  private final StatusSignal<Current> current1;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Current> current2;
  private final StatusSignal<Voltage> appliedVoltsUp;
  private final StatusSignal<Current> currentUp;

  private final Follower followerReq1;

  public IndexerIOTalonFX() {
    motor1 = new TalonFX(IndexerConstants.MOTOR_1_ID, "mainCAN");
    motor2 = new TalonFX(IndexerConstants.MOTOR_2_ID, "mainCAN");
    motorUp = new TalonFX(IndexerConstants.INDEXER_UP_MOTOR_ID, "mainCAN");

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        IndexerConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.ENABLE_SUPPLY_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerTime = IndexerConstants.SUPPLY_CURRENT_LOWER_TIME_SEC;
    cfg.CurrentLimits.StatorCurrentLimitEnable = IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT_AMPS;
    motor1.getConfigurator().apply(cfg);
    motor2.getConfigurator().apply(cfg);

    // IndexerUp 使用相同电流限制，方向根据常量配置
    var cfgUp = new TalonFXConfiguration();
    cfgUp.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfgUp.MotorOutput.Inverted =
        IndexerConstants.INDEXER_UP_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfgUp.CurrentLimits.SupplyCurrentLimitEnable = IndexerConstants.ENABLE_SUPPLY_CURRENT_LIMIT;
    cfgUp.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    cfgUp.CurrentLimits.SupplyCurrentLowerLimit = IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    cfgUp.CurrentLimits.SupplyCurrentLowerTime = IndexerConstants.SUPPLY_CURRENT_LOWER_TIME_SEC;
    cfgUp.CurrentLimits.StatorCurrentLimitEnable = IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT;
    cfgUp.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT_AMPS;
    motorUp.getConfigurator().apply(cfgUp);

    followerReq1 = new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed);
    motor2.setControl(followerReq1);

    appliedVolts1 = motor1.getMotorVoltage();
    current1 = motor1.getStatorCurrent();
    appliedVolts2 = motor2.getMotorVoltage();
    current2 = motor2.getStatorCurrent();
    appliedVoltsUp = motorUp.getMotorVoltage();
    currentUp = motorUp.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVolts1, current1, appliedVolts2, current2, appliedVoltsUp, currentUp);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var status1 = BaseStatusSignal.refreshAll(appliedVolts1, current1);
    var status2 = BaseStatusSignal.refreshAll(appliedVolts2, current2);
    var statusUp = BaseStatusSignal.refreshAll(appliedVoltsUp, currentUp);
    inputs.appliedVolts1 = appliedVolts1.getValueAsDouble();
    inputs.current1Amps = current1.getValueAsDouble();
    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
    inputs.current2Amps = current2.getValueAsDouble();
    inputs.connected = status1.isOK() && status2.isOK();
    inputs.upAppliedVolts = appliedVoltsUp.getValueAsDouble();
    inputs.upCurrentAmps = currentUp.getValueAsDouble();
    inputs.upConnected = statusUp.isOK();
  }

  @Override
  public void setVoltage(double volts) {
    motor1.setControl(voltageReq1.withOutput(volts));
  }

  @Override
  public void setUpVoltage(double volts) {
    motorUp.setControl(voltageReqUp.withOutput(volts));
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    motorUp.stopMotor();
  }
}
