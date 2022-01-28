// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TransferConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Indexer extends SubsystemBase
{
  private Indexer m_indexer = new Indexer();
  private CANSparkMax indexerMotor1;
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;

  public Indexer()
  {
    indexerMotor1 = new CANSparkMax(IndexerConstants.TRANSFER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController1 = indexerMotor1.getPIDController();
    encoder2 = indexerMotor1.getEncoder();
    
    indexerMotor1.setClosedLoopRampRate(TransferConstants.kRampRate);
  }

  public void motor1Position(double position)
  {
    pidController1.setReference(position, ControlType.kPosition);
  }

  public void  motorStop() {
    indexerMotor1.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Indexer getInstance() {
    return m_indexer;
  }

}
