// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IndexerConstants;
// import team3176.robot.subsystems.IndexerIO.IndexerIOInputs;
import team3176.robot.util.God.Units3176;
import team3176.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.I2C;

public class Indexer extends SubsystemBase {
  private static Indexer instance;
  public String mode = "";
  private TalonSRX indexerMotor;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = new boolean[IndexerConstants.NUM_OF_SENSORS];
  private boolean isSmartDashboardTestControlsShown;
  private boolean firstPos, secondPos, thirdPos;
  private boolean lastFirstPos, lastSecondPos, lastThirdPos;
  private int lastState = 222;
  private int currentState = 222;
  private double startingEncoderTic;
  private int ballCount;
  private int mycounter = 0;
  private double smartDashboardLastPercent = 0.0;
  private int lastI2CUpdateLoops;
  private boolean twoMinuteLock;
  private DigitalInput secondLinebreak;

  private Intake m_Intake;

  // private final IndexerIO io;
  // private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;

  private Indexer(/*IndexerIO io*/) {
    // this.io = io;

    lastI2CUpdateLoops = 0;
    twoMinuteLock = false;
    this.currentState = reportState();
    indexerMotor = new TalonSRX(IndexerConstants.INDEXER_CAN_ID);
    ballCount = 0;
    m_Intake = Intake.getInstance();

    m_I2C = new I2C(I2C.Port.kMXP, IndexerConstants.I2C_DEVICE_ADDRESS);

    this.indexerMotor.configAllowableClosedloopError(IndexerConstants.kPID_LOOP_IDX[1],
        IndexerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kF(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][3],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kP(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][0],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kI(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][1],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kD(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][2],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_IntegralZone(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][4],
        IndexerConstants.kTIMEOUT_MS);

    //I2CReciever(); 
     secondLinebreak = new DigitalInput(IndexerConstants.SECOND_LINEBREAK_DIO);
     secondPos = secondLinebreak.get();

  }
  
  public void motorStop() {
    indexerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public int reportState() {
    // I2CReciever();
    int state = 0;
    if (!firstPos)
      state += 100;
    if (!secondPos)
      state += 10;
    if (!thirdPos)
      state++;

    return state;
  }

  public void Up() { // TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void Down() { // TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput, -0.5);
  }

  /**
   * Recieves Line Breaker Data from Uno and Arranges them in a Byte Array.
   * Then it changes the Bytes into Booleans and then puts the values into a
   * Boolean Array
   */

  public void I2CReciever() {
    sensorByteArray = new byte[IndexerConstants.NUM_OF_SENSORS];
    m_I2C.readOnly(sensorByteArray, IndexerConstants.NUM_OF_SENSORS);
    sensorBoolArray = new boolean[sensorByteArray.length];
    for (int i = 0; i < sensorByteArray.length; i++) {
      sensorBoolArray[i] = sensorByteArray[i] != 0;
    }

    if(!twoMinuteLock) {
      this.firstPos = sensorBoolArray[0];
      this.secondPos = sensorBoolArray[1];
      this.thirdPos = sensorBoolArray[2];
    }

    if(secondPos != lastSecondPos) {
      lastI2CUpdateLoops = 0;
    }

    lastI2CUpdateLoops++;
    // if(twoMinuteLock && lastI2CUpdateLoops > 200) {
    //   twoMinuteLock = false;
    // }
    if(lastI2CUpdateLoops >= 6000) { //2 Minutes
      this.secondPos = false;
      this.twoMinuteLock = true;
    }

    // double ticvalue = indexerMotor.getSelectedSensorPosition();
    
    if (mycounter > 100) {
      // System.out.println(secondPos);
      // System.out.println("1: " + firstPos + ", 2: " + secondPos + ", 3: " + thirdPos + ", tics:" + ticvalue);
      mycounter = 0;
    } else {
      mycounter++;
    }

    this.lastFirstPos = this.firstPos;
    this.lastSecondPos = this.secondPos;
    this.lastThirdPos = this.thirdPos;
  }

  public void putSmartDashboardControlCommands() {
    // SmartDashboard.putNumber("Indexer PCT", 0);
    isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    smartDashboardLastPercent = SmartDashboard.getNumber("Indexer PCT", 0);
    indexerMotor.set(ControlMode.PercentOutput, (SmartDashboard.getNumber("Indexer PCT", 0)));
  }

  public void putSmartDashboardControlCommands(double startPercent) {
    SmartDashboard.putNumber("Indexer PCT", startPercent);
  }

  public void setPIDValuesFromSmartDashboard() {
    smartDashboardLastPercent = SmartDashboard.getNumber("Indexer PID PCT", 0);
    indexerMotor.set(ControlMode.Velocity, (SmartDashboard.getNumber("Indexer PID PCT", 0)) * Units3176.revolutionsPerMinute2ticsPer100MS(18730, 4096));
  }

  public void putSmartDashboardPIDControlCommands(double startPercent) {
    SmartDashboard.putNumber("Indexer PID PCT", startPercent);
  }

  public double getStartPercent() {return smartDashboardLastPercent;}

  @Override
  public void periodic() {
    this.secondPos = secondLinebreak.get();

    /*
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Indexer", inputs);
    Logger.getInstance().recordOutput("Indexer/Bool0", sensorBoolArray[0]);
    Logger.getInstance().recordOutput("Indexer/Bool1", sensorBoolArray[1]);
    Logger.getInstance().recordOutput("Indexer/Bool2", sensorBoolArray[2]);
    */

    if (mode.equals("test")) {
      if (!isSmartDashboardTestControlsShown)
        putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
    // lastState = currentState;
    // currentState = reportState();
    // System.out.println(reportState());
    // if((lastState != currentState)) {System.out.println("The Indexer Changed");
    // System.out.println(reportState());
    // }

    // I2CReciever();
    SmartDashboard.putBoolean("Indexer 2nd Line", getSecondPos());
  }

  @Override
  public void simulationPeriodic() {}

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer(/*new IndexerIO() {}*/);
    }
    return instance;
  }

  public void setPCT(double pct) {
    indexerMotor.set(ControlMode.PercentOutput, pct);
  }

  public void setVelocity(double pctAsDecimal) {
    indexerMotor.set(ControlMode.Velocity, pctAsDecimal * Units3176.revolutionsPerMinute2ticsPer100MS(18730, 4096));
  }

  public void simpleIndexer() {if(!secondPos) {motorStop();}}

  public boolean getFirstPos() {return firstPos;}
  public boolean getSecondPos() {return secondPos;}
  public boolean getThirdPos() {return thirdPos;}
}