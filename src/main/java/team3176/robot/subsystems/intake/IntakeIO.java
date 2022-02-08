// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO{
  /** Contains all of the input data received from hardware. */
  public static class IntakeIOInputs implements LoggableInputs {
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public boolean isExtend = false;
    public double[] currentAmps = new double[] {};
    //public double[] tempCelcius = new double[] {};

    public void toLog(LogTable table) {
      table.put("VelocityRadPerSec", velocity);
      table.put("isExtend", isExtend);
      table.put("AppliedVolts", appliedVolts);
      table.put("CurrentAmps", currentAmps);
      //table.put("TempCelcius", tempCelcius);
    }

    public void fromLog(LogTable table) {
      velocity = table.getDouble("Velocity", velocity);
      isExtend = table.getBoolean("isExtend", isExtend);
      appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
      currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
      //tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Run closed loop at the specified velocity.
   * 
   * @param velocityRadPerSec Velocity setpoint.
   * @param ffVolts Feed forward voltage from model.
   */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Retract or Extend pistons */
  public default void setPiston(boolean isExtend) {}

  /** Enable or disable brake mode. */
  //public default void setBrakeMode(boolean enable) {}

  /** Set velocity PID constants. */
  //public default void configurePID(double kp, double ki, double kd) {}

  /** Reset the encoder(s) to a known position. */
  //public default void resetPosition(double positionRad) {}
}