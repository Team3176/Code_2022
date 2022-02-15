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

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import team3176.robot.subsystems.indexer.*;
import team3176.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase
{
  // private static Indexer instance = new Indexer();
  private static Indexer instance;

  private CANSparkMax indexerMotor;
  private SparkMaxPIDController pidController1;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = {false, false};
  private DigitalInput input;

  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;

  private Indexer(IndexerIO io) 
  {
    this.io = io;

    indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController1 = indexerMotor.getPIDController();
    encoder2 = indexerMotor.getEncoder();
    //input = new DigitalInput(0);

    m_I2C = new I2C(I2C.Port.kMXP, 8);
    
    indexerMotor.setClosedLoopRampRate(IndexerConstants.kRampRate);

  }
  public void setIndexerPosition(double position) {
    pidController1.setReference(position, ControlType.kPosition);
  }

  public void  motorStop() {
    indexerMotor.set(0.0);
  }

  /**
   * Recieves Line Breaker Data from Uno and Arranges them in a Byte Array.
   * Then it changes the Bytes into Booleans and then puts the values into a Boolean Array
   */
  public void I2CReciever()
  {
    sensorByteArray = new byte[IndexerConstants.NUM_OF_SENSORS];
    System.out.println("Begin");
    m_I2C.readOnly(sensorByteArray, IndexerConstants.NUM_OF_SENSORS);
    sensorBoolArray = new boolean[sensorByteArray.length];
    for(int i = 0; i < sensorByteArray.length; i++) 
    {
      sensorBoolArray[i] = sensorByteArray[i]!=0;
    }
    /*
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
    */
    
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Indexer", inputs);
    // Logger.getInstance().recordOutput("Indexer/Bool0", sensorBoolArray[0]);
    // Logger.getInstance().recordOutput("Indexer/Bool1", sensorBoolArray[1]);
    // Logger.getInstance().recordOutput("Indexer/Bool2", sensorBoolArray[2]);
    // for(int i = 0; i <= sensorBoolArray.length; i++) {
    //   String key = "Indexer/Bool" + i;
    //   Logger.getInstance().recordOutput(key, sensorBoolArray[i]);
    // }

    // This method will be called once per scheduler run
    // I2CReciever();
    //System.out.println("Input: " + input.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public static Indexer getInstance() {
  //   return instance;
  // }

  public static Indexer getInstance() {
    if(instance == null) {instance = new Indexer(new IndexerIO() {});}
    return instance;
  }

}
