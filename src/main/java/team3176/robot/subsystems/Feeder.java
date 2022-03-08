// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.FeederConstants;
import team3176.robot.subsystems.FeederIO.FeederIOInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger; 

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
public class Feeder extends SubsystemBase
{
  private TalonSRX feederMotor;

  private final FeederIO io;
  private final FeederIOInputs inputs = new FeederIOInputs();
  private static Feeder instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";
  private boolean isFeederRunning = false;

  public Feeder(FeederIO io)
  {
    this.io = io;

    feederMotor = new TalonSRX(FeederConstants.FEEDER_MOTOR_CAN_ID);
    feederMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.feederMotor.configNominalOutputForward(0, FeederConstants.kTIMEOUT_MS);
    this.feederMotor.configNominalOutputReverse(0, FeederConstants.kTIMEOUT_MS);
    this.feederMotor.configPeakOutputForward(0.25, FeederConstants.kTIMEOUT_MS);
    this.feederMotor.configPeakOutputReverse(-0.25, FeederConstants.kTIMEOUT_MS);
    this.feederMotor.configAllowableClosedloopError(FeederConstants.kPID_LOOP_IDX, FeederConstants.ALLOWABLE_CLOSED_LOOP_ERROR, FeederConstants.kTIMEOUT_MS);
    this.feederMotor.config_kF(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][3], FeederConstants.kTIMEOUT_MS);
    this.feederMotor.config_kP(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][0], FeederConstants.kTIMEOUT_MS);
    this.feederMotor.config_kI(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][1], FeederConstants.kTIMEOUT_MS);
    this.feederMotor.config_kD(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][2], FeederConstants.kTIMEOUT_MS);
    this.feederMotor.config_IntegralZone(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][4], FeederConstants.kTIMEOUT_MS);  
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber(FeederConstants.kShuffleboardPercentName, 0.0);
    if (output >= -1 && output <= 1) { feederMotor.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("FeederRPMOut", feederMotor.getSelectedSensorVelocity());
  }

  public void setVelocityPID(double RPM)
  {
    feederMotor.set(ControlMode.Velocity, RPM);
    isFeederRunning = true;
    if(RPM == 0) {isFeederRunning = false;}
  }

  public void stopMotor() {
    feederMotor.set(ControlMode.PercentOutput,0.0);
    isFeederRunning = false;
  }

  public boolean isFeederRunning() {
    return isFeederRunning;
  }

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Feeder Spark PCT", 0);
    isSmartDashboardTestControlsShown = true;
 }

 public void setValuesFromSmartDashboard() {
   feederMotor.set(ControlMode.PercentOutput, (SmartDashboard.getNumber("Feeder PCT", 0)));
 }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.getInstance().processInputs("Feeder", inputs);
    Logger.getInstance().recordOutput("Feeder/Velocity", getFeederVelocity());

    if (mode.equals("test"))
    {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setFeederVelocity(double feederVelocity) 
  {
    io.setFeederVelocity(feederVelocity);
  }

  public double getFeederVelocity()
  {
    return inputs.velocity;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Feeder getInstance() {
    if(instance == null) {instance = new Feeder(new FeederIO() {});}
    return instance;
  }

}
