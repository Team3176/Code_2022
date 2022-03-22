// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IndexerConstants;
import team3176.robot.subsystems.IndexerIO.IndexerIOInputs;
import team3176.robot.util.God.Units3176;
import team3176.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.I2C;

public class Indexer extends SubsystemBase {
  private static Indexer instance;
  public String mode = "";
  private TalonSRX indexerMotor;
  private byte[] sensorByteArray;
  private boolean[] sensorBoolArray = new boolean[IndexerConstants.NUM_OF_SENSORS];
  private boolean isSmartDashboardTestControlsShown;
  private boolean firstPos, secondPos, thirdPos;
  private int lastState = 222;
  private int currentState = 222;
  private double startingEncoderTic;
  private int ballCount;
  private int mycounter = 0;
  private boolean loading = false;
  private boolean holding = false;
  private boolean spitting = false;
  private boolean shooting = false;
  private double smartDashboardLastPercent = 0.0;

  public enum IndexMode {
    LOADING, HOLDING, SPITTING, SHOOTING
  };

  private IndexMode indexMode;

  private Intake m_Intake;

  private final IndexerIO io;
  private final IndexerIOInputs inputs = new IndexerIOInputs();

  private I2C m_I2C;

  private Indexer(IndexerIO io) {
    this.io = io;

    this.currentState = reportState();
    indexerMotor = new TalonSRX(IndexerConstants.INDEXER_CAN_ID);
    setIndexerConfigForPositionPIDCtrl();
    ballCount = 0;
    m_Intake = Intake.getInstance();

    this.indexMode = IndexMode.HOLDING;
    startingEncoderTic = indexerMotor.getSelectedSensorPosition();

    // indexerMotor.setClosedLoopRampRate(IndexerConstants.RAMP_RATE);

    m_I2C = new I2C(I2C.Port.kMXP, IndexerConstants.I2C_DEVICE_ADDRESS);
  }

  public void index() { // HAVE TO SEND BALL COUNT AS A PARAMETER TODO: Find a new way to get ball count
                        // without making it static
    this.ballCount = m_Intake.getBallCount();
    switch (this.indexMode) {
      case LOADING:
        // System.out.println("INDEXER BEGIN LOADING MODE: " + reportState());
        setIndexerConfigForPositionPIDCtrl();
        if (reportState() == 000 && !m_Intake.getPistonSetting() && m_Intake.getMotorSetting()) {
          Up();
          // System.out.println("INDEXER 000 and Piston Retract and Running");
          // Do not run motor
        }
        //if (ballCount == 1) {
          if (reportState() == 100 || reportState() == 000) {
            // PID Position motor up to State010
            indexer000_2_010();
            // System.out.println("INDEXER STATE 100");
          }
          if (reportState() == 010 && indexerMotor.getMotorOutputPercent() > 0) {
            // stopMotor
            motorStop();
            // System.out.println("INDEXER 010 and INDEXER RUNNING");
          }
        //}
        if (ballCount == 2) {
          if (reportState() == 100) {
            // PID Position motor up to State010
            indexer000_2_010();
          }
          if (reportState() == 110) {
            // PID Position motor up to State011
            indexer110_2_011();
          }
        }

        break;
      case HOLDING:
        // System.out.println("INDEXER BEGIN HOLDING MODE");

        motorStop();
      /*
        setIndexerConfigForPositionPIDCtrl();
        if (reportState() == 000) {
          motorStop();
        }

        if (reportState() == 100) {
          // Do nothing
        }
        if (reportState() == 001) {
          // Run motor back to state010
          indexer001_2_010();
        }
        if (reportState() == 011) {
          // Not sure what to do here
        }
        if (reportState() == 101) {
          // This means we fucked up and timing will be wrong for Firing.
          indexer101_2_010();
        }
        if (reportState() == 110) {
          // Not sure what to do here
          indexer110_2_011();
        }
        */
        break;
      case SPITTING:
        // System.out.println("INDEXER BEGIN SPITTING MODE");

        // if (intake is extended) and (intake motor getMotorOutputPercent() < 0) {
        // run motor down at -.80
        // }
        setIndexerConfigForVelocityPIDCtrl();
        if (m_Intake.isExtended()) {
          double target_Spit_RPM = IndexerConstants.MAX_RPM * -0.2;
          double velocity_ticsPer100ms = target_Spit_RPM * IndexerConstants.ENCODER_TICS_PER_REVOLUTION / 600.0;
          indexerMotor.set(ControlMode.Velocity, velocity_ticsPer100ms);
        }
        break;
      case SHOOTING:
        // System.out.println("INDEXER BEGIN SHOOTING MODE");

        setIndexerConfigForVelocityPIDCtrl();
        double target_Shoot_RPM = IndexerConstants.MAX_RPM * 0.4;
        double velocity_ticsPer100ms = target_Shoot_RPM * IndexerConstants.ENCODER_TICS_PER_REVOLUTION / 600.0;
        indexerMotor.set(ControlMode.Velocity, velocity_ticsPer100ms);
        // run motor Up at 80%
        break;
      default:
        break;
    }

  }

  public void setIndexerConfigForPositionPIDCtrl() {
    this.indexerMotor.configAllowableClosedloopError(IndexerConstants.kPID_LOOP_IDX[0],
        IndexerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kF(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][3],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kP(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][0],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kI(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][1],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_kD(IndexerConstants.kPID_LOOP_IDX[0], IndexerConstants.PIDFConstants[0][2],
        IndexerConstants.kTIMEOUT_MS);
    this.indexerMotor.config_IntegralZone(IndexerConstants.kPID_LOOP_IDX[1], IndexerConstants.PIDFConstants[0][4],
        IndexerConstants.kTIMEOUT_MS);
  }

  public void setIndexerConfigForVelocityPIDCtrl() {
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
  }

  public void setIndexerPosition(double position) {
    indexerMotor.set(ControlMode.Position, position);
  }

  public void motorStop() {
    indexerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  // public void IndexerSpin() {
  // indexerMotor.set(0.1);
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

  public void Up() { // TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void Down() { // TODO: RENAME TO SOMETHING BETTER
    indexerMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void requestState(int s) {
    int state = reportState();
    if (state == 100 && s == 110) {
      Up();
    } else {
      while (s < state && reportState() != s) {
        Up();
      }
      while (s > state && reportState() != s) {
        Down();
      }
    }
    if (s == reportState()) {
      motorStop();
    }
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

    this.firstPos = sensorBoolArray[0];
    this.secondPos = sensorBoolArray[1];
    this.thirdPos = sensorBoolArray[2];

    double ticvalue = indexerMotor.getSelectedSensorPosition();
    
    if (mycounter > 100) {
      // System.out.println(secondPos);
      // System.out.println("1: " + firstPos + ", 2: " + secondPos + ", 3: " + thirdPos + ", tics:" + ticvalue);
      mycounter = 0;
    } else {
      mycounter++;
    }
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
    indexerMotor.set(ControlMode.PercentOutput, (SmartDashboard.getNumber("Indexer PID PCT", 0)) * Units3176.revolutionsPerMinute2ticsPer100MS(18730, 4096));
  }

  public void putSmartDashboardPIDControlCommands(double startPercent) {
    SmartDashboard.putNumber("Indexer PID PCT", startPercent);
  }

  public double getStartPercent() {return smartDashboardLastPercent;}

  public void setModeLoading() {
    this.indexMode = IndexMode.LOADING;
    this.loading = true;
    this.holding = false;
    this.spitting = false;
    this.shooting = false;
  }

  public void setModeHolding() {
    this.indexMode = IndexMode.HOLDING;
    this.loading = false;
    this.holding = true;
    this.spitting = false;
    this.shooting = false;
  }

  public void setModeSpitting() {
    this.indexMode = IndexMode.SPITTING;
    this.loading = false;
    this.holding = false;
    this.spitting = true;
    this.shooting = false;
  }

  public void setModeShooting() {
    this.indexMode = IndexMode.SHOOTING;
    this.loading = false;
    this.holding = false;
    this.spitting = false;
    this.shooting = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Indexer", inputs);
    Logger.getInstance().recordOutput("Indexer/Bool0", sensorBoolArray[0]);
    Logger.getInstance().recordOutput("Indexer/Bool1", sensorBoolArray[1]);
    Logger.getInstance().recordOutput("Indexer/Bool2", sensorBoolArray[2]);

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

    I2CReciever();
    SmartDashboard.putBoolean("Indexer 2nd Line", getSecondPos());

  }

  @Override
  public void simulationPeriodic() {
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer(new IndexerIO() {
      });
    }
    return instance;
  }

  public void indexer000_2_010() {
    indexerMotor.set(ControlMode.Position, (indexerMotor.getSelectedSensorPosition() + IndexerConstants.ticDiff_000_010)); // TODO: CHECK SIGN
  }

  public void indexer000_2_011() {
    indexerMotor.set(ControlMode.Position, (indexerMotor.getSelectedSensorPosition() + IndexerConstants.ticDiff_000_011)); // TODO: CHECK SIGN
  }

  public void indexer000_2_001() {
    indexerMotor.set(ControlMode.Position, (indexerMotor.getSelectedSensorPosition() + IndexerConstants.ticDiff_000_001)); // TODO: CHECK SIGN
  }

  public void indexer010_2_110() {
    indexerMotor.set(ControlMode.Position,
        (indexerMotor.getSelectedSensorPosition() + IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_010_110)); // TODO: CHECK SIGN
  }

  public void indexer010_2_111() {
    indexerMotor.set(ControlMode.Position,
        (indexerMotor.getSelectedSensorPosition() + IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_010_111)); // TODO: CHECK SIGN
  }

  public void indexer110_2_011() {
    indexerMotor.set(ControlMode.Position, (indexerMotor.getSelectedSensorPosition() - IndexerConstants.ticDiff_010_110
        - IndexerConstants.ticDiff_000_010 + IndexerConstants.ticDiff_000_011)); // TODO: CHECK SIGNS
  }

  public void indexer001_2_010() {
    indexerMotor.set(ControlMode.Position,
        (indexerMotor.getSelectedSensorPosition() - IndexerConstants.ticDiff_000_001 + IndexerConstants.ticDiff_000_010)); // TODO: CHECK
                                                                                                     // SIGNS
  }

  public void indexer101_2_010(){
    //NO idea what the constants should be here
    //TODO:  KYLE, please measure and fix this method
    //FOr Know it is sudo indexer001_2_010()
    indexer001_2_010();
  }

  public void setPCT(double pct) {
    indexerMotor.set(ControlMode.PercentOutput, pct);
  }

  public void setVelocity(double pctAsDecimal) {
    indexerMotor.set(ControlMode.Velocity, pctAsDecimal * Units3176.revolutionsPerMinute2ticsPer100MS(18730, 4096));
  }

  public void simpleIndexer() {
    // I2CReciever(); //Uncomment if I2CReciever() isn't called in periodic
    if(this.loading) {
      if(!secondPos) {motorStop();}
      // if(reportState() == 010) {
      //   motorStop();
      //   if(m_Intake.ballCount == 2) {return true;}
      // } //else {
      //   Up();
      // }
    }
    
    if(this.holding) {
      motorStop();
    }

    if(this.shooting) {}

    if(this.spitting) {}
  }

  public boolean getFirstPos() {return firstPos;}
  public boolean getSecondPos() {return secondPos;}
  public boolean getThirdPos() {return thirdPos;}
}