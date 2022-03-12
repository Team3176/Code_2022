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
  public String mode = "";
  private TalonSRX indexerMotor;
  private SparkMaxPIDController indexerPIDController;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = new boolean[IndexerConstants.NUM_OF_SENSORS];
  private boolean isSmartDashboardTestControlsShown;
  private boolean firstPos, secondPos, thirdPos;
  private int lastState = 222;
  private int currentState = 222;
  private double startingEncoderTic;

  public enum IndexMode{LOADING, HOLDING, SPITTING, SHOOTING};
  private IndexMode indexMode;

  
  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;


  private Indexer(IndexerIO io) {
    this.io = io;

    this.currentState = reportState();
    indexerMotor = new TalonSRX(IndexerConstants.INDEXER_CAN_ID);
    indexerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.indexerMotor.configNominalOutputForward(0, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.configNominalOutputReverse(0, IndexerConstants.kTIMEOUT_MS);
    // this.indexerMotor.configPeakOutputForward(0.8, IndexerConstants.kTIMEOUT_MS);
    // this.indexerMotor.configPeakOutputReverse(-0.8, IndexerConstants.kTIMEOUT_MS);
    

    this.indexMode = IndexMode.HOLDING;
    startingEncoderTic = indexerMotor.getSelectedSensorPosition();

    //indexerMotor.setClosedLoopRampRate(IndexerConstants.RAMP_RATE);


    m_I2C = new I2C(I2C.Port.kMXP, IndexerConstants.I2C_DEVICE_ADDRESS);
  }

  public void index(int ballCount){ //HAVE TO SEND BALL COUNT AS A PARAMETER TODO: Find a new way to get ball count without making it static
    switch (this.indexMode) {
      case LOADING:
        setIndexerConfigForPositionPIDCtrl();
        if (ballCount == 1) {
          if (reportState() == 000) {
            //Run motor continuously Up
          }
          if (reportState() == 100) {
            //PID Position motor up to State010
            indexer000_2_010();
          }
          if (reportState() == 010 && indexerMotor.getMotorOutputPercent() > 0) {
            //stopMotor
            motorStop();
          }
        } else if(ballCount == 2) {
          if (reportState() == 110 && indexerMotor.getMotorOutputPercent() == 0) {
            //PID Position motor up to State011
            indexer110_2_011();
          }
        }

        break;
      case HOLDING:
        setIndexerConfigForPositionPIDCtrl();
        if (reportState() == 100) {
          //Run motor up to state010
        }
        if (reportState() == 001) {
          //Run motor back to state010
          indexer001_2_010();
        }
        if (reportState() == 011) {
          //Not sure what to do here
        }
        if (reportState() == 101) {
          //This is the state we want to hold in for 2 balls
        }
        if (reportState() == 110) {
          //Not sure what to do here
        }
        break;
      case SPITTING:
        // if (intake is extended) and (intake motor getMotorOutputPercent() < 0) {
            // run motor down at -.80
        //}
        break;
      case SHOOTING:
        setIndexerConfigForVelocityPIDCtrl();
        double target_RPM = IndexerConstants.MAX_RPM * 0.8; 
        double velocity_ticsPer100ms =  target_RPM * IndexerConstants.ENCODER_TICS_PER_REVOLUTION / 600.0;
        indexerMotor.set(ControlMode.Velocity, velocity_ticsPer100ms);
        //run motor Up at 80%
        break;
      default:
        break;
    }


  }


  public void setIndexerConfigForPositionPIDCtrl() {
    this.indexerMotor.configAllowableClosedloopError(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kF(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][3], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kP(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][0], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kI(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][1], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kD(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][2], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_IntegralZone(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[0][4], IndexerConstants.kTIMEOUT_MS); 
  }
  
  public void setIndexerConfigForVelocityPIDCtrl() {
    this.indexerMotor.configAllowableClosedloopError(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kF(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][3], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kP(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][0], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kI(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][1], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kD(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][2], IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_IntegralZone(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[1][4], IndexerConstants.kTIMEOUT_MS); 
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

  public void Up() { //TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput,0.8);
  }

  public void Down() { //TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput,-0.8);
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
    
    this.firstPos = sensorBoolArray[0];
    this.secondPos = sensorBoolArray[1];
    this.thirdPos = sensorBoolArray[2];

    double ticvalue = indexerMotor.getSelectedSensorPosition();

    System.out.println("1: " + firstPos + ", 2: " + secondPos + ", 3: " + thirdPos + ", tics:" + ticvalue);
  }

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Indexer PCT", 0);
    isSmartDashboardTestControlsShown = true;
  }

  public void setValuesFromSmartDashboard() {
    indexerMotor.set(ControlMode.PercentOutput, (SmartDashboard.getNumber("Indexer PCT", 0)));
  }

  public void setModeLoading(){
    this.indexMode = IndexMode.LOADING;
  }
  public void setModeHolding(){
    this.indexMode = IndexMode.HOLDING;
  }
  public void setModeSpitting(){
    this.indexMode = IndexMode.SPITTING;
  }
  public void setModeShooting(){
    this.indexMode = IndexMode.SHOOTING;
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
    // lastState = currentState;
    // currentState = reportState();
    // System.out.println(reportState());
    // if((lastState != currentState)) {System.out.println("The Indexer Changed");
    //  System.out.println(reportState());
    // }

  }

  @Override
  public void simulationPeriodic() {}

  public static Indexer getInstance() {
    if(instance == null) {instance = new Indexer(new IndexerIO() {});}
    return instance;
  }

  public void indexer000_2_010() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic + IndexerConstants.ticDiff_000_010)); //TODO: CHECK SIGN
  }

  public void indexer000_2_011() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic + IndexerConstants.ticDiff_000_011)); //TODO: CHECK SIGN
  }

  public void indexer000_2_001() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic + IndexerConstants.ticDiff_000_001)); //TODO: CHECK SIGN
  }

  public void indexer010_2_110() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic + IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_010_110)); //TODO: CHECK SIGN
  }

  public void indexer010_2_111() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic + IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_010_111)); //TODO: CHECK SIGN
  }

  public void indexer110_2_011() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic - IndexerConstants.ticDiff_010_110 - IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_000_011)); //TODO: CHECK SIGNS
  }

  public void indexer001_2_010() {
    indexerMotor.set(ControlMode.Position, (startingEncoderTic - IndexerConstants.ticDiff_000_001 + IndexerConstants.ticDiff_000_010)); //TODO: CHECK SIGNS
  }
}