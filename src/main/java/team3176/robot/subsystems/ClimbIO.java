// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface ClimbIO{
  /** Contains all of the input data received from hardware. */
  public static class ClimbIOInputs implements LoggableInputs {
    public boolean extended = false;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};
    public boolean limitSwitchOne = false;
    public boolean limitSwitchTwo = false;

    public void toLog(LogTable table) {
      table.put("Extended", extended);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
      table.put("LimitSwitchOne", limitSwitchOne);
      table.put("LimitSwitchTwo", limitSwitchTwo);
    }

    public void fromLog(LogTable table) {
      extended = table.getBoolean("extended", extended);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
      limitSwitchOne = table.getBoolean("LimitSwitchOne", limitSwitchOne);
      limitSwitchTwo = table.getBoolean("LimitSwitchTwo", limitSwitchTwo);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Pistons */

  public default void passivePistonsEngage() {}
  public default void passivePistonsRetract() {}
}