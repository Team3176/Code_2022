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

    public void toLog(LogTable table) {
      table.put("Climb/Extended", extended);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
    }

    public void fromLog(LogTable table) {
      extended = table.getBoolean("Climb/Extended", extended);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Pistons */

  public default void climbPistonsEngage() {}
  public default void climbPistonsRetract() {}
}