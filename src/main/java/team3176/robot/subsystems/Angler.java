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
  // This value is whatever value we have set the motor to, and we don't care what the ControlType is.
  private double setValue;

  // used for Shuffleboard velocity + limit switch testing
  private double smartdashboardVelocity;

   // represents the angle of the Angler at which the encoder should read zero
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
    setValue = 0;

    // used for Shuffleboard velocity + limit switch testing
    // smartdashboardVelocity = 0.0;
    // SmartDashboard.putNumber("Velocity (RPM)", smartdashboardVelocity);
    // SmartDashboard.putNumber("Angler FF", 0.0);
  }

  /**
   * Should be called in this subsystem before any line that makes the motor move. This enables the PID controller after it is
   * automatically disabled when setting the motor to 0.0 speed with the simple set() method.
   */
  public void reengagePIDLoop()
  {
    // Yes, this DOES set velocity control. However, this line is only called to reengage the PID loop before any of the PID position
    // control lines are called. This line will only be active for an instant. The reason for this is because if the set() method is
    // called on the motor, it stops using the PID loop as reference. When the PID loop is called for a reference the next time, the motor
    // may instantly try to jump to whatever speed or position it needs to be at. This is necessary for velocity control, but it may
    // not be necessary for position control. Will have to test.
    //PIDController.setReference(encoder.getVelocity(), ControlType.kVelocity);

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
      this.setValue = value;
      if (!PIDLoopEngaged) { this.reengagePIDLoop(); }
      //PIDController.setReference(value, ControlType.kPosition);
      anglerMotor.set(ControlMode.Position, value);
    }
  }

  public void engageRawMotor(double percentOutput)
  {
    this.setValue = percentOutput;
    if (!limiterHighEngaged && !limiterLowEngaged && percentOutput >= -1 && percentOutput <= 1) {
      PIDLoopEngaged = false;
      anglerMotor.set(ControlMode.PercentOutput ,percentOutput);
    }
  }

  public void testPercentOutput() 
  {
    // engageRawMotor(SmartDashboard.getNumber(AnglerConstants.kShuffleboardPercentName, 0.0));
    // SmartDashboard.putNumber("AnglerRPMOut", anglerMotor.getSelectedSensorVelocity(AnglerConstants.kPID_LOOP_IDX));
  }

  // Does the same thing as engageRawMotor(), except it ignores the limiter conditional so it can stop the motors no matter what
  public void limiterStopMotor()
  {
    PIDLoopEngaged = false;
    // Do NOT change the this.setValue variable for the limiter stopping the motor. It needs to remain where
    // it was so that we know which direction we were trying to go originally, and so that the motor doesn't
    // periodically stop and start over and over while the limit switch is held
    anglerMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void changeAngle(double angleChange)
  {
    double rotationsOfMotor = angleChange * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO;
    if ((anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX) + rotationsOfMotor <= 80 * AnglerConstants.ROTATIONS_PER_DEGREE) &&
        (anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX) + rotationsOfMotor >= 45 * AnglerConstants.ROTATIONS_PER_DEGREE)) {
      this.engagePIDMotorPosition(rotationsOfMotor);
    }
  }

  public void moveToAngle(double newAngle)
  {
    double oldAngleInRotationsOfMotor = anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX);
    double differenceInRotations = (positionAt45Deg + (newAngle * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO)) - oldAngleInRotationsOfMotor;
    if ((newAngle <= 80) && (newAngle >= 45)) {
      this.engagePIDMotorPosition(differenceInRotations);
    }
  }


  /**
   * Sets the velocity of the angler in degrees/sec
   * @param velocity deg/s
   * @author Jared Brown and Christian Schweitzer
   */
  public void setVelocity(double degreesPerSecond)
  {
    double rotationsPerMin = degreesPerSecond * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO * 60;
    //this.engagePIDMotor(rotationsPerMin, ControlType.kVelocity);
  }



  public void zeroAtMin() {
    anglerMotor.setSelectedSensorPosition(0.0, AnglerConstants.kPID_LOOP_IDX, AnglerConstants.kTIMEOUT_MS);
    motorZero = AnglerConstants.kAnglerMinDegrees;
  }

  public void zeroAtMax() {
    anglerMotor.setSelectedSensorPosition(0.0, AnglerConstants.kPID_LOOP_IDX, AnglerConstants.kTIMEOUT_MS);
    motorZero = AnglerConstants.kAnglerMaxDegrees;
  }

  /**
   * Returns a value of the angle above the horizontal at which the Angler's encoder should read zero. This zero position can be changed
   * by running the command that sets the angler's zero position to either its minimum angle or maximum angle. If 0 is returned, the encoder
   * has not been properly zeroed out and therefore has no reference point for position setting -- zero it out first.
   * @return
   */
  public double getAnglerZero() {return motorZero;}


  /*
  Proposed method for finding the position of the angler when robot starts (aka move to 45 degrees and set that as starting position
  to use as a reference):

    A command would do this. This command would initially call a method in this subsystem to set the velocity of the motor to the number
    of degrees per second we want it to move backwards toward the 45 degree boundary. Then, this command constantly checks, using the
    encoder, whether or not the motor is moving. The code to be written in periodic() of this subsystem will stop the motors when they
    reach that limit switch. When the motors stop, the command that is running will end, and that command's end() method will call
    a setter to be written in this subsystem that will set the position that the motor reads in rotations at that moment as the position
    that corresponds to a 45 degree angle. Then, this can be used as a reference for setting other angles.

    See method below \/\/\/
  */

  public void setPositionAt45Deg()
  {
    positionAt45Deg = anglerMotor.getSelectedSensorPosition(AnglerConstants.kPID_LOOP_IDX);
  }

  public void shuffleboardVelocity()  
  {

    // double newVelocity = SmartDashboard.getNumber("Velocity (RPM)", this.smartdashboardVelocity);
    // if (newVelocity != this.smartdashboardVelocity) 
    // {
    //   this.smartdashboardVelocity = newVelocity;
    //   //engagePIDMotor(this.smartdashboardVelocity, ControlType.kVelocity);
    // }
  }

  public void tunePID()
  {
    // double newFF = SmartDashboard.getNumber("Angler FF", 0.0);
    // if (newFF != this.lastFF) {    
    //   this.anglerMotor.config_kF(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[3], AnglerConstants.kTIMEOUT_MS);
    //   this.lastFF = newFF;
    // }
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


  public void setFF(double newFF) {
      this.anglerMotor.config_kF(AnglerConstants.kPID_LOOP_IDX, AnglerConstants.PIDFConstants[3], AnglerConstants.kTIMEOUT_MS);
  }

  public void putSmartDashboardControlCommands() {
    // SmartDashboard.putNumber("Angler Position", 0);
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

    // When pressed, DigitalInput.get() returns FALSE!!! (makes total sense)
    if (!limitSwitch1.get() && setValue < 0) {
      limiterStopMotor();
      limiterLowEngaged = true;
      limiterHighEngaged = false;
      // System.out.println("LEFT LIMITER PRESSED ---------------");
    } else if (!limitSwitch2.get() && setValue > 0) {
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
