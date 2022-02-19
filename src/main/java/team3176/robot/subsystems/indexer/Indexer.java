// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IndexerConstants;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.I2C;
import team3176.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase
{
  // private static Indexer instance = new Indexer();
  private static Indexer instance;

  private CANSparkMax indexerMotor;
  private SparkMaxPIDController indexerPIDController;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = new boolean[IndexerConstants.NUM_OF_SENSORS];
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";

  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;

  private Indexer(IndexerIO io) {
    this.io = io;

    indexerMotor = new CANSparkMax(IndexerConstants.INDEXER_CAN_ID, MotorType.kBrushless);
    indexerMotor.setClosedLoopRampRate(IndexerConstants.RAMP_RATE);

    indexerPIDController = indexerMotor.getPIDController();

    m_I2C = new I2C(I2C.Port.kMXP, IndexerConstants.I2C_DEVICE_ADDRESS);
  }

  public void setIndexerPosition(double position) {
    indexerPIDController.setReference(position, ControlType.kPosition);
  }

  public void  motorStop() {
    indexerMotor.set(0.0);
  }

  /**
   * Recieves Line Breaker Data from Uno and Arranges them in a Byte Array.
   * Then it changes the Bytes into Booleans and then puts the values into a Boolean Array
   */

  public void I2CReciever() {
    sensorByteArray = new byte[IndexerConstants.NUM_OF_SENSORS];
    m_I2C.readOnly(sensorByteArray, IndexerConstants.NUM_OF_SENSORS);
    sensorBoolArray = new boolean[sensorByteArray.length];
    for(int i = 0; i < sensorByteArray.length; i++) {
      sensorBoolArray[i] = sensorByteArray[i]!=0;
    }
  }

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Indexer PCT", 0);
    isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    indexerMotor.set(SmartDashboard.getNumber("Indexer PCT", 0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Indexer", inputs);
    Logger.getInstance().recordOutput("Indexer/Bool0", sensorBoolArray[0]);
    Logger.getInstance().recordOutput("Indexer/Bool1", sensorBoolArray[1]);

    if(mode.equals("test")) {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

  @Override
  public void simulationPeriodic() {}

  public static Indexer getInstance() {
    if(instance == null) {instance = new Indexer(new IndexerIO() {});}
    return instance;
  }
}