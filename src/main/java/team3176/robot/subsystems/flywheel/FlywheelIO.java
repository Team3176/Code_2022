// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.flywheel;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface FlywheelIO{
  /** Contains all of the input data received from hardware. */
  public static class FlywheelIOInputs implements LoggableInputs {
    public double velocity_1 = 0.0;
    public double velocity_2 = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Velocity 1", velocity_1);
      table.put("Velocity 2", velocity_2);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity_1 = table.getDouble("Velocity 1", velocity_1);
      velocity_1 = table.getDouble("Velocity 2", velocity_2);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Encoder Position of the Indexer */

  public default void setFlywheelVelocity1(double velocity_1) {}

  public default void setFlywheelVelocity2(double velocity_2) {}
}
