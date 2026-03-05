package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IndexerIO {
  public static class IndexerIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double appliedVolts1 = 0.0;
    public double current1Amps = 0.0;
    public double appliedVolts2 = 0.0;
    public double current2Amps = 0.0;
    // IndexerUp
    public boolean upConnected = false;
    public double upAppliedVolts = 0.0;
    public double upCurrentAmps = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("AppliedVolts1", appliedVolts1);
      table.put("Current1Amps", current1Amps);
      table.put("AppliedVolts2", appliedVolts2);
      table.put("Current2Amps", current2Amps);
      table.put("UpConnected", upConnected);
      table.put("UpAppliedVolts", upAppliedVolts);
      table.put("UpCurrentAmps", upCurrentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      appliedVolts1 = table.get("AppliedVolts1", appliedVolts1);
      current1Amps = table.get("Current1Amps", current1Amps);
      appliedVolts2 = table.get("AppliedVolts2", appliedVolts2);
      current2Amps = table.get("Current2Amps", current2Amps);
      upConnected = table.get("UpConnected", upConnected);
      upAppliedVolts = table.get("UpAppliedVolts", upAppliedVolts);
      upCurrentAmps = table.get("UpCurrentAmps", upCurrentAmps);
    }
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Sets indexer output voltage (volts). */
  public default void setVoltage(double volts) {}

  /** Sets IndexerUp output voltage (volts). Independent of main indexer. */
  public default void setUpVoltage(double volts) {}

  public default void stop() {}
}
