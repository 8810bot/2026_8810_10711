// Copyright 2025
package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem for the shared hood. Both shooters use this single hood. */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  private double hoodSetpointDeg = 0.0;
  private double hoodVelSetpointDegPerSec = 0.0;

  private boolean isHomed = false;

  private enum HomingState {
    IDLE,
    HOMING_DOWN,
    HOLDING,
    SETTLING,
    ENGAGING
  }

  private HomingState homingState = HomingState.IDLE;

  private double stateStartTime = 0.0;
  private static final double HOMING_VOLTS = -2.0;
  private static final double HOMING_CURRENT_THRESHOLD = 5.0;
  private static final double HOMING_SURGE_DELAY = 0.1;
  private static final double HOLD_DURATION = 0.2;
  private static final double HOLD_VOLTS = -1.0;
  private static final double SETTLING_DURATION = 2.0;
  private static final double ENGAGING_DURATION = 0.5;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    double timeInState = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;

    switch (homingState) {
      case HOMING_DOWN:
        io.setVoltage(HOMING_VOLTS);
        if (timeInState > HOMING_SURGE_DELAY
            && inputs.hoodCurrentAmps >= HOMING_CURRENT_THRESHOLD) {
          homingState = HomingState.HOLDING;
          stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        break;

      case HOLDING:
        io.setVoltage(HOLD_VOLTS);
        if (timeInState > HOLD_DURATION) {
          homingState = HomingState.SETTLING;
          stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
          io.setVoltage(0.0); // 停止施加电压，让齿轮系统回弹释放间隙
        }
        break;

      case SETTLING:
        io.setVoltage(0.0);
        if (timeInState > SETTLING_DURATION) {
          homingState = HomingState.ENGAGING;
          stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
          // 第一次清零：切入闭环前必须先告诉电机“当前位置就是0”，防止它疯转去历史的0度
          io.zeroPosition();
          // 激活闭环 PID（目标定为现在的0度），此时PID内部启动激流（KS/KG等）会吃满齿轮间隙
          io.setHoodAngleDeg(0.0, 0.0);
        }
        break;

      case ENGAGING:
        // 等待 PID 填充完毕齿轮间隙的空载移动...
        if (timeInState > ENGAGING_DURATION) {
          isHomed = true;
          homingState = HomingState.IDLE;
          io.zeroPosition(); // 在齿隙被彻底顶死（吃撑）且稳定后的瞬间，宣布这就是 0 度
          hoodSetpointDeg = 0.0;
          hoodVelSetpointDegPerSec = 0.0;
          Logger.recordOutput("Hood/HomingSuccess", true);
        }
        break;

      case IDLE:
        if (isHomed) {
          io.setHoodAngleDeg(hoodSetpointDeg, hoodVelSetpointDegPerSec);
        } else {
          io.stop();
        }
        break;
    }

    Logger.recordOutput("Hood/SetpointDeg", hoodSetpointDeg);
    Logger.recordOutput("Hood/VelSetpointDegPerSec", hoodVelSetpointDegPerSec);
    Logger.recordOutput("Hood/IsHomed", isHomed);
    Logger.recordOutput("Hood/HomingState", homingState.toString());
  }

  /** Sets hood/backplate angle (degrees). */
  public void setAngle(double deg) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = 0.0;
  }

  /** Sets hood/backplate angle with velocity feedforward (deg and deg/s). */
  public void setAngle(double deg, double velDegPerSec) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = velDegPerSec;
  }

  public void stop() {
    hoodVelSetpointDegPerSec = 0.0;
    io.stop();
  }

  /** Zeroes the hood position by running a homing sequence. */
  public void zeroPosition() {
    homingState = HomingState.HOMING_DOWN;
    isHomed = false;
    stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    Logger.recordOutput("Hood/HomingSuccess", false);
  }

  /** Returns hood/backplate angle (mechanism degrees). */
  public double getAngleDeg() {
    return inputs.hoodAngleDeg;
  }

  /** Returns whether hood IO is connected/healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
