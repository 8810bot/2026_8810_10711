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

  private final VoltageOut voltageReq1 = new VoltageOut(0.0);
  private final VoltageOut voltageReq2 = new VoltageOut(0.0);

  private final StatusSignal<Voltage> appliedVolts1;
  private final StatusSignal<Current> current1;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Current> current2;

  private final Follower followerReq1;

  public IndexerIOTalonFX() {
    motor1 = new TalonFX(IndexerConstants.MOTOR_1_ID, "mainCAN");
    motor2 = new TalonFX(IndexerConstants.MOTOR_2_ID, "mainCAN");

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        IndexerConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.CurrentLimits.SupplyCurrentLimitEnable =
        IndexerConstants.ENABLE_SUPPLY_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = IndexerConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerLimit =
        IndexerConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerTime =
        IndexerConstants.SUPPLY_CURRENT_LOWER_TIME_SEC;
    cfg.CurrentLimits.StatorCurrentLimitEnable =
        IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.StatorCurrentLimit = IndexerConstants.STATOR_CURRENT_LIMIT_AMPS;
    motor1.getConfigurator().apply(cfg);
    motor2.getConfigurator().apply(cfg);

    followerReq1 = new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed);
    motor2.setControl(followerReq1);

    appliedVolts1 = motor1.getMotorVoltage();
    current1 = motor1.getStatorCurrent();
    appliedVolts2 = motor2.getMotorVoltage();
    current2 = motor2.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, appliedVolts1, current1, appliedVolts2, current2);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var status1 = BaseStatusSignal.refreshAll(appliedVolts1, current1);
    var status2 = BaseStatusSignal.refreshAll(appliedVolts2, current2);
    inputs.appliedVolts1 = appliedVolts1.getValueAsDouble();
    inputs.current1Amps = current1.getValueAsDouble();
    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
    inputs.current2Amps = current2.getValueAsDouble();
    inputs.connected = status1.isOK() && status2.isOK();
  }

  @Override
  public void setVoltage(double volts) {
    motor1.setControl(voltageReq1.withOutput(volts));
  }

  @Override
  public void stop() {
    motor1.stopMotor();
  }
}
