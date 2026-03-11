// Copyright 2026
package frc.robot.subsystems.hopper;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import frc.robot.constants.HopperConstants;

/** Hopper IO that combines CANrange sensing with REV Servo Hub CAN servo control. */
public class HopperIOServo extends HopperIOCANrange {
  private static final int SERVO_COUNT = 5;
  private static final int SERVO_MIN_PULSE_WIDTH_US = 500;
  private static final int SERVO_CENTER_PULSE_WIDTH_US = 1500;
  private static final int SERVO_MAX_PULSE_WIDTH_US = 2500;
  private static final int SERVO_PERIOD_US = 20_000;

  private final ServoHub servoHub = new ServoHub(HopperConstants.SERVO_HUB_CAN_ID);
  private final ServoChannel[] servos = new ServoChannel[SERVO_COUNT];

  public HopperIOServo() {
    configureServoHubChannels();

    // 上电时只使能通道，不发送脉宽指令，舵机保持当前物理位置
    for (int index = 0; index < servos.length; index++) {
      ServoChannel channel = servoHub.getServoChannel(ServoChannel.ChannelId.fromInt(index));
      channel.setPowered(true);
      channel.setEnabled(true);
      servos[index] = channel;
    }
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.servoHubConnected = isServoHubConnected();
    inputs.connected = inputs.connected && inputs.servoHubConnected;
    inputs.servo1Position = pulseWidthToPosition(servos[0].getPulseWidth());
    inputs.servo2Position = pulseWidthToPosition(servos[1].getPulseWidth());
    inputs.servo3Position = pulseWidthToPosition(servos[2].getPulseWidth());
    inputs.servo4Position = pulseWidthToPosition(servos[3].getPulseWidth());
    inputs.servo5Position = pulseWidthToPosition(servos[4].getPulseWidth());
  }

  @Override
  public void setServoPosition(int index, double position) {
    if (index < 0 || index >= servos.length) {
      return;
    }
    servos[index].setEnabled(true);
    servos[index].setPowered(true);
    servos[index].setPulseWidth(positionToPulseWidth(clamp(position)));
  }

  private static double clamp(double value) {
    return Math.max(0.0, Math.min(1.0, value));
  }

  private static int positionToPulseWidth(double position) {
    return (int)
        Math.round(
            SERVO_MIN_PULSE_WIDTH_US
                + clamp(position) * (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US));
  }

  private static double pulseWidthToPosition(int pulseWidthUs) {
    return clamp(
        (pulseWidthUs - SERVO_MIN_PULSE_WIDTH_US)
            / (double) (SERVO_MAX_PULSE_WIDTH_US - SERVO_MIN_PULSE_WIDTH_US));
  }

  private boolean isServoHubConnected() {
    try {
      return servoHub.getDeviceVoltage() > 0.01 || servoHub.getServoVoltage() > 0.01;
    } catch (Exception ignored) {
      return false;
    }
  }

  private void configureServoHubChannels() {
    servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, SERVO_PERIOD_US);
    servoHub.setBankPulsePeriod(ServoHub.Bank.kBank3_5, SERVO_PERIOD_US);

    ServoHubConfig config = new ServoHubConfig();
    config
        .channel0
        .pulseRange(SERVO_MIN_PULSE_WIDTH_US, SERVO_CENTER_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US)
        .pulseRange(500, 1500, 2500)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel1
        .pulseRange(500, 1500, 2500)
        .pulseRange(SERVO_MIN_PULSE_WIDTH_US, SERVO_CENTER_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel2
        .pulseRange(500, 1500, 2500)
        .pulseRange(SERVO_MIN_PULSE_WIDTH_US, SERVO_CENTER_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel3
        .pulseRange(500, 1500, 2500)
        .pulseRange(SERVO_MIN_PULSE_WIDTH_US, SERVO_CENTER_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    config
        .channel4
        .pulseRange(500, 1500, 2500)
        .pulseRange(SERVO_MIN_PULSE_WIDTH_US, SERVO_CENTER_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    servoHub.configure(config, ResetMode.kResetSafeParameters);
  }
}
