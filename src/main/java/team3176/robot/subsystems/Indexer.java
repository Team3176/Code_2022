// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IndexerConstants;
import team3176.robot.subsystems.IndexerIO.IndexerIOInputs;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.I2C;

public class Indexer extends SubsystemBase {
  private static Indexer instance;

  private TalonSRX indexerMotor;
  private SparkMaxPIDController indexerPIDController;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = new boolean[IndexerConstants.NUM_OF_SENSORS];
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";
  private boolean firstPos, secondPos, thirdPos;

  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;

  private Indexer(IndexerIO io) {
    this.io = io;

    indexerMotor = new TalonSRX(IndexerConstants.INDEXER_CAN_ID);
    indexerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.indexerMotor.configNominalOutputForward(0, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.configNominalOutputReverse(0, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.configPeakOutputForward(0.25, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.configPeakOutputReverse(-0.25, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.configAllowableClosedloopError(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kF(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.PIDFConstants[3], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kP(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.PIDFConstants[0], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kI(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.PIDFConstants[1], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kD(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.PIDFConstants[2], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_IntegralZone(IndexerConstants.kPID_LOOP_IDX, IndexerConstants.PIDFConstants[4], IndexerConstants.kTIMEOUT_MS); 



    //indexerMotor.setClosedLoopRampRate(IndexerConstants.RAMP_RATE);


    m_I2C = new I2C(I2C.Port.kMXP, IndexerConstants.I2C_DEVICE_ADDRESS);
  }

  public void setIndexerPosition(double position) {
    indexerMotor.set(ControlMode.Position, position);
  }

  public void motorStop() {
    indexerMotor.set(ControlMode.PercentOutput,0.0);
  }

  // public void IndexerSpin() {
  //   indexerMotor.set(0.1);
  // }

  public int reportState() {
    int state = 0;
    if (firstPos)
      state += 100;
    if (secondPos)
      state += 10;
    if (thirdPos)
      state++;
    
    return state;
  }

  public void Up() { //TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput,0.1);
  }

  public void Down() { //TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput,-0.1);
  }

  public void requestState(int s) {
    int state = reportState();
    if(state == 100 && s == 110) {
      Up();
    } else {
      while (s < state && reportState() != s) {
        Up();
      }
      while (s > state && reportState() != s) {
        Down();
      }
    }
    if(s == reportState()) {motorStop();}
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
    
    firstPos = sensorBoolArray[0];
    secondPos = sensorBoolArray[1];
    thirdPos = sensorBoolArray[2];

  }

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Indexer PCT", 0);
    isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    indexerMotor.set(ControlMode.PercentOutput, (SmartDashboard.getNumber("Indexer PCT", 0)));
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