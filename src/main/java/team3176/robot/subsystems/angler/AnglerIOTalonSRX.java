// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.angler;

import org.littletonrobotics.junction.LogTable;

/** Template hardware interface for a closed loop subsystem. */
public class AnglerIOTalonSRX implements AnglerIO{
  /** Contains all of the input data received from hardware. */
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("Position", positionRad);
      table.put("VelocityRadPerSec", velocityRadPerSec);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      positionRad = table.getDouble("PositionRad", positionRad);
      velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    inputs.positionRad = 0;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {}

  /** Encoder Position of the Indexer */
  @Override
  public void setAnglerPosition(double position) {}
}
