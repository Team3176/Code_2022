// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.FlywheelConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Flywheel extends SubsystemBase {

  private static Flywheel m_flywheel = new Flywheel();
  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  public Flywheel()
  {
    flywheelMotor1 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON1_CAN_ID);
    flywheelMotor2 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON2_CAN_ID);

    flywheelMotor1.configFactoryDefault();
    flywheelMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.setSensorPhase(true);
    flywheelMotor1.configClosedloopRamp(FlywheelConstants.kRampRate, FlywheelConstants.kTimeoutMS);

    flywheelMotor2.configFactoryDefault();
    flywheelMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor2.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    // This will (hopefully) invert the second motor
    flywheelMotor2.setSensorPhase(false);
    flywheelMotor2.configClosedloopRamp(FlywheelConstants.kRampRate, FlywheelConstants.kTimeoutMS);
    
    flywheelMotor1.config_kP(0, FlywheelConstants.PIDFConstants[0][0]);
    flywheelMotor1.config_kI(0, FlywheelConstants.PIDFConstants[0][1]);
    flywheelMotor1.config_kD(0, FlywheelConstants.PIDFConstants[0][2]);
    flywheelMotor1.config_kF(0, FlywheelConstants.PIDFConstants[0][3]);
    flywheelMotor1.config_IntegralZone(0, FlywheelConstants.PIDFConstants[0][4]);

    flywheelMotor1.config_kP(0, FlywheelConstants.PIDFConstants[0][0]);
    flywheelMotor1.config_kI(0, FlywheelConstants.PIDFConstants[0][1]);
    flywheelMotor1.config_kD(0, FlywheelConstants.PIDFConstants[0][2]);
    flywheelMotor1.config_kF(0, FlywheelConstants.PIDFConstants[0][3]);
    flywheelMotor1.config_IntegralZone(0, FlywheelConstants.PIDFConstants[0][4]);
  }

  public void spinMotors(double ticksPer100ms) {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  public void spinMotors(double ticksPer100msForMotor1, double ticksPer100msForMotor2) {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100msForMotor1);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100msForMotor2);
  }

  /*
  public void spinMotors2(double metersPerSecond)
  {
    // double ticsPer100ms = --MATH!-- (will need radius of flywheel for v = r(omega))
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }
  */

  public void percentOutput_1() 
  {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor1.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly1Tics/100msOut", flywheelMotor1.getSelectedSensorVelocity());
  }

  public void percentOutput_2() 
  {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName2, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor2.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly2Tics/100msOut", flywheelMotor2.getSelectedSensorVelocity());
  }

  public void stopMotors()
  {
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Flywheel getInstance() {
    return m_flywheel;
  }
  
}
