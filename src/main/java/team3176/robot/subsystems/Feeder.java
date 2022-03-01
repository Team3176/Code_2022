// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.FeederConstants;
import team3176.robot.subsystems.FeederIO.TransferIOInputs;
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
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;

  private final FeederIO io;
  private final TransferIOInputs inputs = new TransferIOInputs();
  private static Feeder instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";

  public Feeder(FeederIO io)
  {
    this.io = io;

    feederMotor = new CANSparkMax(FeederConstants.TRANSFER_NEO_CAN_ID, MotorType.kBrushless);
    pidController1 = feederMotor.getPIDController();
    encoder1 = feederMotor.getEncoder();
    
    feederMotor.setClosedLoopRampRate(FeederConstants.kRampRate);
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber(FeederConstants.kShuffleboardPercentName, 0.0);
    if (output >= -1 && output <= 1) { feederMotor.set(output); }
    SmartDashboard.putNumber("TransferRPMOut", encoder1.getVelocity());
  }

  public void motor2Velocity(double velocity)
  {
    pidController1.setReference(velocity, ControlType.kVelocity);
  }

  public void stopMotor() {
    feederMotor.set(0.0);
  }

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Feeder Spark PCT", 0);
    isSmartDashboardTestControlsShown = true;
 }

 public void setValuesFromSmartDashboard() {
   feederMotor.set(SmartDashboard.getNumber("Feeder Spark PCT", 0));
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
