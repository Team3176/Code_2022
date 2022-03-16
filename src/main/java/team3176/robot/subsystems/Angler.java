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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Angler extends SubsystemBase {
  
  private TalonSRX anglerMotor;

  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;

  private int ticsPerRevolution;
  private double positionAt45Deg;

  private boolean PIDLoopEngaged;
  private boolean limiterHighEngaged;
  private boolean limiterLowEngaged;
  private boolean isSmartDashboardTestControlsShown;

  private double lastFF;

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

  // used for Shuffleboard velocity + limit switch testing
  private double smartdashboardVelocity;

   // represents the angle of the Angler at which the encoder should read zero -- this initializes to zero to ensure the Angler is zeroed
   // before the motor is commanded to go somewhere
   private double motorZero;

 public Angler(AnglerIO io) 
 {
   this.io = io;

    this.anglerMotor = new TalonSRX(AnglerConstants.ANGLER_SPARK_CAN_ID);
    anglerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
    this.anglerMotor.configNominalOutputForward(0, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configNominalOutputReverse(0, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configPeakOutputForward(0.25, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configPeakOutputReverse(-0.25, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.configAllowableClosedloopError(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.ALLOWABLE_CLOSED_LOOP_ERROR, AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kF(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[3], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kP(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[0], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kI(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[1], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_kD(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[2], AnglerConstants.kTIMEOUT_MS);
    this.anglerMotor.config_IntegralZone(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[4], AnglerConstants.kTIMEOUT_MS); 
    this.lastFF = AnglerConstants.PIDFConstants[3];
    // stop the motor on enable
    limitSwitch1 = new DigitalInput(AnglerConstants.limiter1Channel);
    limitSwitch2 = new DigitalInput(AnglerConstants.limiter2Channel);

    PIDLoopEngaged = false;
    limiterHighEngaged = false;
    limiterLowEngaged = false;
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

  /**
   * Should be called in this subsystem before any line that makes the motor move. This enables the PID controller after it is
   * automatically disabled when setting the motor to 0.0 speed with the simple set() method.
   */
  public void reengagePIDLoop()
  {
    // Yes, this DOES set velocity control. However, this line is only called to reengage the PID loop before any of the PID position
    // control lines are called. This line will only be active for an instant. The reason for this is because if the set() method is
    // called on the motor with ControlMode.Percent Output, it may stop using the PID loop as reference. When the PID loop is called for a
    // reference the next time, the motor may instantly try to jump to whatever speed or position it needs to be at. This is necessary for
    // velocity control, but it may not be necessary for position control. Will have to test. This is an EXCEPTION to the rule that the
    // set() method should never be used.
    anglerMotor.set(ControlMode.Velocity, anglerMotor.getSelectedSensorVelocity());
    PIDLoopEngaged = true;
  }

  /**
   * Should be called anytime the motor should be set in this subsystem instead of writing the set line. This method accounts for the possibility of the
   * PID loop being disengaged from setting the raw speed using the simple set() method and the limiter being engaged.
   * @param value
   * @param controlType
   * @author Jared Brown
   */
  public void engagePIDMotorPosition(double value)
  {
    if (!limiterHighEngaged && !limiterLowEngaged) {
      this.setMovement = value - anglerMotor.getSelectedSensorPosition();
      if (!PIDLoopEngaged) { this.reengagePIDLoop(); }
      anglerMotor.set(ControlMode.Position, value);
    }
  }

  public void engagePIDMotorVelocity(double value)
  {
    if (!limiterHighEngaged && !limiterLowEngaged) {
      this.setMovement = value;
      if (!PIDLoopEngaged) { this.reengagePIDLoop(); }
      anglerMotor.set(ControlMode.Velocity, value);
    }
  }

  public void engageRawMotor(double percentOutput)
  {
    this.setMovement = percentOutput;
    if (!limiterHighEngaged && !limiterLowEngaged && percentOutput >= -1 && percentOutput <= 1) {
      PIDLoopEngaged = false;
      anglerMotor.set(ControlMode.PercentOutput ,percentOutput);
    }
  }

  public void testPercentOutput() 
  {
    engageRawMotor(SmartDashboard.getNumber(AnglerConstants.kShuffleboardPercentName, 0.0));
    SmartDashboard.putNumber("AnglerRPMOut", anglerMotor.getSelectedSensorVelocity(AnglerConstants.kPID_LOOP_IDX));
  }

  // Does the same thing as engageRawMotor(), except it ignores the limiter conditional so it can stop the motors no matter what
  public void limiterStopMotor()
  {
    PIDLoopEngaged = false;
    // Do NOT change the this.setMovement variable for the limiter stopping the motor. It needs to remain where
    // it was so that we know which direction we were trying to go originally, and so that the motor doesn't
    // periodically stop and start over and over while the limit switch is held. This is an EXCEPTION to the rule that the
    // set() method should never be used.
    anglerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Moves the angler to the angle specified in degrees.
   * @param newAngle degrees
   * @author Jared Brown
   */
  public void moveToAngle(double newAngle)
  {
    if (newAngle >= AnglerConstants.kAnglerMinDegrees && newAngle <= AnglerConstants.kAnglerMaxDegrees) {
      double angleInTicsPastMinimum = (newAngle - AnglerConstants.kAnglerMinDegrees) * AnglerConstants.TICS_PER_DEGREE;
      if (motorZero == AnglerConstants.kAnglerMinDegrees) {
        this.engagePIDMotorPosition(angleInTicsPastMinimum);
      } else if (motorZero == AnglerConstants.kAnglerMaxDegrees) {
        this.engagePIDMotorPosition(angleInTicsPastMinimum - AnglerConstants.MIN_MAX_TIC_DIFFERENCE);
      }
    }
  }

  /**
   * Raises or lowers the angler by the given number of degrees. Positive is higher, negative is lower.
   * @param angleChange degrees
   * @author Jared Brown
   */
  public void changeAngle(double angleChange)
  {
    double changeInTics = angleChange * AnglerConstants.TICS_PER_DEGREE;
    this.engagePIDMotorPosition(anglerMotor.getSelectedSensorPosition() + changeInTics);
  }

  /**
   * Sets the velocity of the angler in degrees/sec
   * @param velocity deg/s
   * @author Jared Brown and Christian Schweitzer
   */
  public void setVelocity(double degreesPerSecond)
  {
    double rotationsPerMin = degreesPerSecond * AnglerConstants.ROTATIONS_OF_MOTOR_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO * AnglerConstants.TICS_PER_REVOLUTION / 10;
    this.engagePIDMotorVelocity(rotationsPerMin);
  }

  public void zeroAtMin() {
    anglerMotor.setSelectedSensorPosition(0.0, AnglerConstants.kPID_LOOP_IDX, AnglerConstants.kTIMEOUT_MS);
    motorZero = AnglerConstants.kAnglerMinDegrees;
    anglerMotor.setSelectedSensorPosition(0);
  }

  public void zeroAtMax() {
    anglerMotor.setSelectedSensorPosition(0.0, AnglerConstants.kPID_LOOP_IDX, AnglerConstants.kTIMEOUT_MS);
    motorZero = AnglerConstants.kAnglerMaxDegrees;
    anglerMotor.setSelectedSensorPosition(0);
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
  public boolean getLimiterLow() {return limiterLowEngaged;}
    /**
   * Returns true if the higher limit switch is pressed
   * @author Jared Brown
   * @return Higher limiter value
   */
  public boolean getLimiterHigh() {return limiterHighEngaged;}

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
    //System.out.println(!limitSwitch1.get() + ", " + !limitSwitch2.get());

    // limiter stuff, DON'T MODIFY
    double outputPercent = anglerMotor.getMotorOutputPercent();
    if (outputPercent > 0) {
      previousPercentDirection = 1;
    } else if (outputPercent < 0) {
      previousPercentDirection = -1;
    }

    // When pressed, DigitalInput.get() returns FALSE!!! (makes total sense)
    if (!limitSwitch1.get() && (setMovement < 0 || outputPercent < 0 || previousPercentDirection == -1)) {
      limiterStopMotor();
      limiterLowEngaged = true;
      limiterHighEngaged = false;
      // System.out.println("LEFT LIMITER PRESSED ---------------");
    } else if (!limitSwitch2.get() && (setMovement > 0 || outputPercent > 0 || previousPercentDirection == 1)) {
      limiterStopMotor();
      limiterHighEngaged = true;
      limiterLowEngaged = false;
      // System.out.println("RIGHT LIMITER PRESSED ---------------");
    } else {
      limiterLowEngaged = false;
      limiterHighEngaged = false;
    }
    // if(mode.equals("test"))
    // {
    //   if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
    //   setValuesFromSmartDashboard();
    // }

    // Safety checks to make sure the motor is not being powered while limit switches are engaged
    SmartDashboard.putNumber("AnglerVoltage", anglerMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("AnglerPctOutput", anglerMotor.getMotorOutputPercent());
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
