// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.FlywheelConstants;
import team3176.robot.subsystems.FlywheelIO.FlywheelIOInputs;
import team3176.robot.util.God.Units3176;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends SubsystemBase {

  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private static Flywheel instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";
  private boolean isFlywheelSpinning = false;
  private double smartDashboardLastPercent1 = 0.0;
  private double smartDashboardLastPercent2 = 0.0;

  public Flywheel(FlywheelIO io) {
    this.io = io;

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

    flywheelMotor2.config_kP(0, FlywheelConstants.PIDFConstants[1][0]);
    flywheelMotor2.config_kI(0, FlywheelConstants.PIDFConstants[1][1]);
    flywheelMotor2.config_kD(0, FlywheelConstants.PIDFConstants[1][2]);
    flywheelMotor2.config_kF(0, FlywheelConstants.PIDFConstants[1][3]);
    flywheelMotor2.config_IntegralZone(0, FlywheelConstants.PIDFConstants[1][4]);

    flywheelMotor1.setInverted(true);
    flywheelMotor2.setInverted(true);
  }

  public void spinMotors(double ticksPer100ms) {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
    isFlywheelSpinning = true;
    if(ticksPer100ms == 0) {isFlywheelSpinning = false;}
  }

  public void spinMotors(double ticksPer100msForMotor1, double ticksPer100msForMotor2) {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100msForMotor1);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100msForMotor2);
    isFlywheelSpinning = true;
    if((ticksPer100msForMotor1 == 0) && (ticksPer100msForMotor2 == 0)) {isFlywheelSpinning = false;}
  }

  public void spinMotorsVelocityPID(double pctOne, double pctTwo) {
    flywheelMotor1.set(TalonFXControlMode.Velocity, pctOne * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
    flywheelMotor2.set(TalonFXControlMode.Velocity, pctTwo * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
    isFlywheelSpinning = true;
    if((pctOne == 0) && (pctTwo == 0)) {isFlywheelSpinning = false;}
  }

  public boolean getMotorSpinning() {return isFlywheelSpinning;}
  /*
  public void spinMotors2(double metersPerSecond) {
    // double ticsPer100ms = --MATH!-- (will need radius of flywheel for v = r(omega))
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }
  */

  public void percentOutput_1() {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor1.set(TalonFXControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly1Tics/100msOut", flywheelMotor1.getSelectedSensorVelocity());
  }

  public void percentOutput_2() {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName2, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor2.set(TalonFXControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly2Tics/100msOut", flywheelMotor2.getSelectedSensorVelocity());
  }

  public void stopMotors() {
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, 0.0);
    isFlywheelSpinning = false;
  }

  public void setPCT(double pct1, double pct2) {
    isFlywheelSpinning = true;
    flywheelMotor1.set(ControlMode.PercentOutput, pct1);
    flywheelMotor2.set(ControlMode.PercentOutput, pct2);
  }

  public void putSmartDashboardControlCommands() {
    // SmartDashboard.putNumber("Flywheel 1 PCT", 0);
    // SmartDashboard.putNumber("Flywheel 2 PCT", 0);
    isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    smartDashboardLastPercent1 = SmartDashboard.getNumber("Flywheel 1 PCT", 0);
    smartDashboardLastPercent2 = SmartDashboard.getNumber("Flywheel 2 PCT", 0);
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Flywheel 1 PCT", 0));
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Flywheel 2 PCT", 0));
  }

  public void putSmartDashboardControlCommands(double startPercent1, double startPercent2) {
    SmartDashboard.putNumber("Flywheel 1 PCT", startPercent1);
    SmartDashboard.putNumber("Flywheel 2 PCT", startPercent2);
  }

  public void putSmartDashboardPIDControlCommands(double startPercent1, double startPercent2) {
    SmartDashboard.putNumber("Flywheel 1 PID PCT", startPercent1);
    SmartDashboard.putNumber("Flywheel 2 PID PCT", startPercent2);
  }

  public void setPIDValuesFromSmartDashboard() {
    smartDashboardLastPercent1 = SmartDashboard.getNumber("Flywheel 1 PID PCT", 0);
    smartDashboardLastPercent2 = SmartDashboard.getNumber("Flywheel 2 PID PCT", 0);
    flywheelMotor1.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Flywheel 1 PID PCT", 0) * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
    flywheelMotor2.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("Flywheel 2 PID PCT", 0) * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
  }

  public double getStartPercent1() {return smartDashboardLastPercent1;}
  public double getStartPercent2() {return smartDashboardLastPercent2;}

  public void stopWithPCT() {
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, 0);
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    Logger.getInstance().processInputs("Flywheel", inputs);
    Logger.getInstance().recordOutput("Flywheel/Velocity 1", getFlywheelVelocity1());
    Logger.getInstance().recordOutput("Flywheel/Velocity 2", getFlywheelVelocity2());

    if(mode.equals("test")) {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getFlywheelVelocity1() {
    return inputs.velocity_1;
  }
  
  public double getFlywheelVelocity2() {
    return inputs.velocity_2;
  }

  public void setFlywheelVelocity1(double velocity) {
    io.setFlywheelVelocity1(velocity);
  }

  public void setFlywheelVelocity2(double velocity) {
    io.setFlywheelVelocity2(velocity);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Flywheel getInstance() {
    if(instance == null) {instance = new Flywheel(new FlywheelIO() {});}
    return instance;
  } 
}