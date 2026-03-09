// Copyright 2025
package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.HoodConstants;
import frc.robot.util.LoggedTunableNumber;

/** TalonFX implementation of HoodIO. Shared by both shooters. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  private final MotionMagicTorqueCurrentFOC hoodMotionMagicReq =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final VoltageOut voltageReq = new VoltageOut(0.0);

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.MOTOR_ID, "mainCAN");

    var hoodCfg = new TalonFXConfiguration();
    hoodCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodCfg.MotorOutput.Inverted =
        HoodConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    hoodCfg.Feedback.SensorToMechanismRatio = HoodConstants.SENSOR_TO_MECH_RATIO;
    hoodCfg.Slot0 =
        new Slot0Configs()
            .withKP(HoodConstants.KP.get())
            .withKI(HoodConstants.KI.get())
            .withKD(HoodConstants.KD.get())
            .withKS(HoodConstants.KS.get())
            .withKV(HoodConstants.KV.get())
            .withKA(HoodConstants.KA.get())
            .withKG(HoodConstants.KG.get());
    hoodCfg.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(HoodConstants.MM_CRUISE_VELOCITY.get())
            .withMotionMagicAcceleration(HoodConstants.MM_ACCELERATION.get())
            .withMotionMagicJerk(HoodConstants.MM_JERK.get());
    hoodMotor.getConfigurator().apply(hoodCfg);
    hoodMotor.setPosition(0);

    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          var slot0 = new Slot0Configs();
          hoodMotor.getConfigurator().refresh(slot0);
          slot0.kP = HoodConstants.KP.get();
          slot0.kI = HoodConstants.KI.get();
          slot0.kD = HoodConstants.KD.get();
          slot0.kS = HoodConstants.KS.get();
          slot0.kV = HoodConstants.KV.get();
          slot0.kA = HoodConstants.KA.get();
          slot0.kG = HoodConstants.KG.get();
          hoodMotor.getConfigurator().apply(slot0);

          var mm = new MotionMagicConfigs();
          hoodMotor.getConfigurator().refresh(mm);
          mm.MotionMagicCruiseVelocity = HoodConstants.MM_CRUISE_VELOCITY.get();
          mm.MotionMagicAcceleration = HoodConstants.MM_ACCELERATION.get();
          mm.MotionMagicJerk = HoodConstants.MM_JERK.get();
          hoodMotor.getConfigurator().apply(mm);
        },
        HoodConstants.KP,
        HoodConstants.KI,
        HoodConstants.KD,
        HoodConstants.KS,
        HoodConstants.KV,
        HoodConstants.KA,
        HoodConstants.KG,
        HoodConstants.MM_CRUISE_VELOCITY,
        HoodConstants.MM_ACCELERATION,
        HoodConstants.MM_JERK);

    var status =
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);
    inputs.hoodAngleDeg = hoodPosition.getValueAsDouble() * 360.0;
    inputs.hoodVelocityDegPerSec = hoodVelocity.getValueAsDouble() * 360.0;
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
    inputs.connected = status.isOK();
  }

  @Override
  public void setHoodAngleDeg(double deg) {
    double rotations = deg / 360.0;
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void setHoodAngleDeg(double deg, double velDegPerSec) {
    double rotations = deg / 360.0;
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageReq.withOutput(volts));
  }

  @Override
  public void zeroPosition() {
    hoodMotor.setPosition(0.0);
  }

  @Override
  public void stop() {
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(0));
  }
}
