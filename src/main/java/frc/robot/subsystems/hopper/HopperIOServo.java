// Copyright 2026
package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.PWM;

/** Hopper IO that combines CANrange sensing with PWM servo control. */
public class HopperIOServo extends HopperIOCANrange {
  private final PWM[] servos = {new PWM(0), new PWM(1), new PWM(2), new PWM(3), new PWM(4)};

  public HopperIOServo() {
    for (PWM servo : servos) {
      // max, deadbandMax, center, deadbandMin, min (microseconds)
      servo.setBoundsMicroseconds(2500, 1500, 1500, 1500, 500);
      servo.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
      servo.setPosition(0.5);
    }
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    super.updateInputs(inputs);
    inputs.servo1Position = servos[0].getPosition();
    inputs.servo2Position = servos[1].getPosition();
    inputs.servo3Position = servos[2].getPosition();
    inputs.servo4Position = servos[3].getPosition();
    inputs.servo5Position = servos[4].getPosition();
  }

  @Override
  public void setServoPosition(int index, double position) {
    if (index < 0 || index >= servos.length) {
      return;
    }
    servos[index].setPosition(clamp(position));
  }

  private static double clamp(double value) {
    return Math.max(0.0, Math.min(1.0, value));
  }
}
