// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IndexerConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;

public class Indexer extends SubsystemBase
{
  private static Indexer instance = new Indexer();
  private CANSparkMax indexerMotor;
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray;

  private I2C m_I2C;

  public Indexer()
  {
    indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController1 = indexerMotor.getPIDController();
    encoder2 = indexerMotor.getEncoder();

    m_I2C = new I2C(I2C.Port.kMXP, 8);
    
    indexerMotor.setClosedLoopRampRate(IndexerConstants.kRampRate);

  }
  public void indexer1Position(double position)
  {
    pidController1.setReference(position, ControlType.kPosition);
  }

  public void  motorStop() {
    indexerMotor.set(0.0);
  }

  public void I2CReciever()
  {
    sensorByteArray = new byte[2];
    System.out.println("Begin");
    m_I2C.readOnly(sensorByteArray, 2);
    sensorBoolArray = new boolean[sensorByteArray.length];
    for(int i = 0; i < sensorByteArray.length; i++) 
    {
      sensorBoolArray[i] = sensorByteArray[i]!=0;
    }
    System.out.println(sensorBoolArray);
    for (int i = 0; i < sensorBoolArray.length; i++)
    {
      if (sensorBoolArray[i])
      {
        System.out.println("Enabled " + i);
      }
      else if (!sensorBoolArray[i])
      {
        System.out.println("Disabled " + i);
      }
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    // I2CReciever();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Indexer getInstance() {
    return instance;
  }

}
