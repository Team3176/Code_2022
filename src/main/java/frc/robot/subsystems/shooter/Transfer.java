// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TransferConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Transfer extends SubsystemBase
{
  private Transfer m_transfer = new Transfer();
  private CANSparkMax transferMotor1;
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;

  public Transfer()
  {
    transferMotor1 = new CANSparkMax(TransferConstants.TRANSFER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController1 = transferMotor1.getPIDController();
    encoder1 = transferMotor1.getEncoder();
    
    transferMotor1.setClosedLoopRampRate(TransferConstants.kRampRate);
  }

  public void motor2Velocity(double velocity)
  {
    pidController1.setReference(velocity, ControlType.kVelocity);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Transfer getInstance() {
    return m_transfer;
  }

}
