// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.indexer;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface IndexerIO{
  /** Contains all of the input data received from hardware. */
  public static class IndexerIOInputs implements LoggableInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean Bool0, Bool1, Bool2;

    public void toLog(LogTable table) {
      table.put("Position", position);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("Indexer/Bool0", Bool0);
      table.put("Indexer/Bool1", Bool1);
      table.put("Indexer/Bool2", Bool2);
    }

    public void fromLog(LogTable table) {
      position = table.getDouble("Position", position);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      Bool0 = table.getBoolean("Indexer/Bool0", Bool0);
      Bool1 = table.getBoolean("Indexer/Bool1", Bool1);
      Bool2 = table.getBoolean("Indexer/Bool2", Bool2);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setIndexerPCT(double pct) {}

  /** Encoder Position of the Indexer */
  public default void setIndexerPosition(double position) {}
}