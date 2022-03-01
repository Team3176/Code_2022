// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.TransferConstants;
import team3176.robot.subsystems.FeederIO.FeederIOInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger; 

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Feeder extends SubsystemBase
{
  private CANSparkMax feederMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  private final FeederIO io;
  private final FeederIOInputs inputs = new FeederIOInputs();
  private static Feeder instance;

  public Feeder(FeederIO io)
  {
    this.io = io;

    feederMotor = new CANSparkMax(TransferConstants.TRANSFER_NEO_CAN_ID, MotorType.kBrushless);
    pidController = feederMotor.getPIDController();
    encoder = feederMotor.getEncoder();
    
    feederMotor.setClosedLoopRampRate(TransferConstants.kRampRate);
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber(TransferConstants.kShuffleboardPercentName, 0.0);
    if (output >= -1 && output <= 1) { feederMotor.set(output); }
    SmartDashboard.putNumber("TransferRPMOut", encoder.getVelocity());
  }

  public void setVelocityPID(double RPM)
  {
    pidController.setReference(RPM, ControlType.kVelocity);
  }

  public void stopMotor() {
    feederMotor.set(0.0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.getInstance().processInputs("Feeder", inputs);
    Logger.getInstance().recordOutput("Feeder/Velocity", getFeederVelocity());
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
