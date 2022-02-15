// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.TransferConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import team3176.robot.subsystems.shooter.TransferIO.TransferIOInputs;

public class Transfer extends SubsystemBase
{
  private CANSparkMax transferMotor1;
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;

  private final TransferIO io;
  private final TransferIOInputs inputs = new TransferIOInputs();
  private static Transfer instance;

  public Transfer(TransferIO io)
  {
    this.io = io;

    transferMotor1 = new CANSparkMax(TransferConstants.TRANSFER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController1 = transferMotor1.getPIDController();
    encoder1 = transferMotor1.getEncoder();
    
    transferMotor1.setClosedLoopRampRate(TransferConstants.kRampRate);
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber(TransferConstants.kShuffleboardPercentName, 0.0);
    if (output >= -1 && output <= 1) { transferMotor1.set(output); }
    SmartDashboard.putNumber("TransferRPMOut", encoder1.getVelocity());
  }

  public void motor2Velocity(double velocity)
  {
    pidController1.setReference(velocity, ControlType.kVelocity);
  }

  public void stopMotor() {
    transferMotor1.set(0.0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Transfer getInstance() {
    if(instance == null) {instance = new Transfer(new TransferIO() {});}
    return instance;
  }

}
