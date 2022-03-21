// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.AnglerConstants;
import team3176.robot.subsystems.AnglerIO.AnglerIOInputs;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Angler extends SubsystemBase {
  
  private TalonSRX anglerMotor;

  private DigitalInput bottomLimiter;
  private DigitalInput topLimiter;

  private boolean isSmartDashboardTestControlsShown;

  private final AnglerIO io;
  private final AnglerIOInputs inputs = new AnglerIOInputs();
  private static Angler instance;
  public String mode = "";

  // Used to help us allow the angler to be touching a limit switch and move away from it. This value is only used for its sign.
  // This value is whatever value we have set the motor to for speed or a CHANGE in position.
  private double setMovement;

  // An extra layer of limiter safety. Once it is set from zero, it only has two possible values: -1 and 1. -1 represents the negative
  // direction, and 1 is the positive direction. This value is set automatically in periodic when the motor tries to rotate in either
  // direction, and is ONLY changed when the motor tries to CHANGE direction (so NOT altered when the motor is commanded 0%). It is to help
  // the limiters remember where the motor is trying to go if it is ever commanded without using engagePIDMotor methods or engageRawMotor.
  private int previousPercentDirection;

  private double intent;

  // used for Shuffleboard velocity + limit switch testing
  private double smartdashboardVelocity;

   // represents the angle of the Angler at which the encoder should read zero -- this initializes to zero to ensure the Angler is zeroed
   // before the motor is commanded to go somewhere
   private double motorZero;
   private boolean hasMotorBeenZeroedAtMax;

  public Angler(AnglerIO io) 
  {
    this.io = io;

    this.hasMotorBeenZeroedAtMax = false;
    this.intent = 0;

    this.anglerMotor = new TalonSRX(AnglerConstants.ANGLER_SPARK_CAN_ID);
    anglerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.anglerMotor.configNominalOutputForward(0, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configNominalOutputReverse(0, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configPeakOutputForward(AnglerConstants.kMaxPercentOutput, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configPeakOutputReverse(-AnglerConstants.kMaxPercentOutput, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configAllowableClosedloopError(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kP(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[0], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kI(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[1], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kD(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[2], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_IntegralZone(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[3], AnglerConstants.kTIMEOUT_MS); 
    
    // anglerMotor.configForwardSoftLimitThreshold(0, AnglerConstants.kTIMEOUT_MS);
    // anglerMotor.configReverseSoftLimitThreshold(-30000, AnglerConstants.kTIMEOUT_MS);
    
    // stop the motor on enable
    bottomLimiter = new DigitalInput(AnglerConstants.limiter1Channel);
    topLimiter = new DigitalInput(AnglerConstants.limiter2Channel);

    setMovement = 0;
    motorZero = 0;
    previousPercentDirection = 0;

    // used for Shuffleboard velocity + limit switch testing
    // smartdashboardVelocity = 0.0;
    // SmartDashboard.putNumber("Velocity (RPM)", smartdashboardVelocity);
    // SmartDashboard.putNumber("Angler FF", 0.0);

    // Safety checks to make sure the motor is not being powered while limit switches are engaged
    SmartDashboard.putNumber("AnglerVoltage", anglerMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("AnglerPctOutput", anglerMotor.getMotorOutputPercent());
  }


  // Returns the intention of our setting of the motor - which direction we want it to go. 1 is positive intent, -1 is negative
  // intent, 0 is intent to not move, and 2 means we shouldn't call the specified ControlMode on the motor as it's not been
  // accounted for properly
  private int determineIntent(ControlMode mode, double set)
  {
    if (mode == ControlMode.PercentOutput || mode == ControlMode.Velocity || mode == ControlMode.Current) {
      if (set > 0) {
        return 1;
      } else if (set < 0) {
        return -1;
      } else {
        return 0;
      }
    } else if (mode == ControlMode.Position) {
      if (set - anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX) > 0) {
        return 1;
      } else if (set - anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX) < 0) {
        return -1;
      } else {
        return 0;
      }
    } else {
      return 2;
    }
  }

  // Called by the engageMotor methods in this class. The engageMotor methods are accessed from outside commands, and that
  // method accesses this one to actually set the motor
  private void setMotorWithLimiterBound(ControlMode mode, double set)
  {
    if (topLimiter.get() && bottomLimiter.get()) {
      anglerMotor.set(mode, set);
    } else if (!topLimiter.get() && (this.intent == -1 || intent == 0) && bottomLimiter.get()) {
      anglerMotor.set(mode, set);
    } else if (!bottomLimiter.get() && (this.intent == 1 || this.intent == 0) && topLimiter.get()) {
      anglerMotor.set(mode, set);
    } else {
      anglerMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  // Called by periodic() in this class to handle or lessen the damage of any limiter edge cases. Might be unnecessary but
  // unsure. Lag might also be a problem but unsure.
  private void periodicLimiterCheck(double voltage)
  {
    if (topLimiter.get() && bottomLimiter.get()) {
      // do nothing
    } else if (!topLimiter.get() && voltage <= 0 && bottomLimiter.get()) {
      // do nothing
    } else if (!bottomLimiter.get() && voltage >= 0 && topLimiter.get()) {
      // do nothing
    } else {
      anglerMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  // Sets the motor to an exact position in tics
  public void engagePIDMotorPosition(double ticPosition)
  {
    if (hasMotorBeenZeroedAtMax) {
      this.intent = determineIntent(ControlMode.Position, ticPosition);
      setMotorWithLimiterBound(ControlMode.Position, ticPosition);
    }
  }

  // Sets the velocity of the motor in tics per 100ms
  public void engagePIDMotorVelocity(double tics100ms)
  {
    this.intent = determineIntent(ControlMode.Velocity, tics100ms);
    setMotorWithLimiterBound(ControlMode.Velocity, tics100ms);
  }

  // sets the motor to a percent output
  public void engageRawMotor(double percentOutput)
  {
    this.intent = determineIntent(ControlMode.PercentOutput, percentOutput);
    setMotorWithLimiterBound(ControlMode.PercentOutput, percentOutput);
  }

  public void testPercentOutput() 
  {
    engageRawMotor(SmartDashboard.getNumber(AnglerConstants.kShuffleboardPercentName, 0.0));
    SmartDashboard.putNumber("AnglerTicsPer100msOut", anglerMotor.getSelectedSensorVelocity(AnglerConstants.kPID_LOOP_IDX));
  }


  /**
   * Moves the angler to the angle specified in degrees.
   * @param newAngle degrees
   * @author Jared Brown
   */
  public void moveToAngle(double newAngle)
  {
    if (newAngle >= AnglerConstants.kAnglerMinDegrees && newAngle <= AnglerConstants.kAnglerMaxDegrees) {
      this.engagePIDMotorPosition(-(AnglerConstants.kAnglerMaxDegrees - newAngle) * AnglerConstants.TICS_OF_MOTOR_PER_DEGREE);
    }
  }

  /**
   * Raises or lowers the angler by the given number of degrees. Positive is higher, negative is lower.
   * @param angleChange degrees
   * @author Jared Brown
   */
  public void changeAngle(double angleChange)
  {
    double changeInTics = angleChange * AnglerConstants.TICS_OF_MOTOR_PER_DEGREE;
    this.engagePIDMotorPosition(anglerMotor.getSelectedSensorPosition() + changeInTics);
  }

  /**
   * Sets the velocity of the angler in degrees per second
   * @param velocity deg/s
   * @author Jared Brown and Christian Schweitzer
   */
  public void setVelocity(double degreesPerSecond)
  {
    double ticsPer100ms = degreesPerSecond * AnglerConstants.TICS_OF_MOTOR_PER_DEGREE / 10;
    this.engagePIDMotorVelocity(ticsPer100ms);
  }

  public void zeroAtMax() {
    this.anglerMotor.setSelectedSensorPosition(0.0, AnglerConstants.kPID_LOOP_IDX, AnglerConstants.kTIMEOUT_MS);
    this.motorZero = AnglerConstants.kAnglerMaxDegrees;
    this.hasMotorBeenZeroedAtMax = true;
  }

  /**
   * Returns a value of the angle above the horizontal at which the Angler's encoder should read zero. This zero position can be changed
   * by running the command that sets the angler's zero position to either its minimum angle or maximum angle. If 0 is returned, the encoder
   * has not been properly zeroed out and therefore has no reference point for position setting -- zero it out first using the
   * AnglerZeroAtMin or AnglerZeroAtMax commands.
   * @return
   */
  public double getAnglerZero() {return motorZero;}


  public void shuffleboardVelocity()  
  {
    double newVelocity = SmartDashboard.getNumber("Velocity (RPM)", this.smartdashboardVelocity);
    if (newVelocity != this.smartdashboardVelocity) 
    {
      this.smartdashboardVelocity = newVelocity;
      // double velocityInTicsPer100ms = MATH...
      // engagePIDMotorVelocity(this.smartdashboardVelocity);
    }
  }

   /**
   * Returns true if the lower limit switch is pressed
   * @author Jared Brown
   * @return Lower limiter value
   */
  public boolean getBottomLimit() {return !bottomLimiter.get();}
    /**
   * Returns true if the higher limit switch is pressed
   * @author Jared Brown
   * @return Higher limiter value
   */
  public boolean getTopLimit() {return !topLimiter.get();}


  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Angler Position", 0);
    isSmartDashboardTestControlsShown = true;
  }

  // public void setValuesFromSmartDashboard() {
  //   this.anglerMotor.set(ControlMode.Position, (SmartDashboard.getNumber("Angler Position", 0)));
  // }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.getInstance().processInputs("Angler", inputs);
    Logger.getInstance().recordOutput("Angler/Position", getAnglerPosition());
    //System.out.println(!bottomLimiter.get() + ", " + !topLimiter.get());

    this.periodicLimiterCheck(anglerMotor.getMotorOutputVoltage());

    // if(mode.equals("test"))
    // {
    //   if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
    //   setValuesFromSmartDashboard();
    // }

    // Safety checks to make sure the motor is not being powered while limit switches are engaged
    SmartDashboard.putNumber("AnglerVoltage", anglerMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("AnglerPctOutput", anglerMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("AnglerEncoderPos", anglerMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("AnglerTopLimitPressed", !topLimiter.get());
    SmartDashboard.putBoolean("AnglerBottomLimitPressed", !bottomLimiter.get());
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getAnglerPosition()
  {
    return inputs.position;
  }

  public void setAnglerPosition(double position)
  {
    io.setAnglerPosition(position);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Angler getInstance() {
    if(instance == null) {instance = new Angler(new AnglerIO() {});}
    return instance;
  }

}
