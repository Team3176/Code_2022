// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.AnglerConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Angler extends SubsystemBase {
  
  private static Angler m_Angler = new Angler();
  private CANSparkMax anglerMotor;
  private SparkMaxPIDController PIDController;
  private RelativeEncoder encoder;

  private double positionAt45Deg;
  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;
  private boolean PIDLoopEngaged;
  private boolean limiter1Engaged;
  private boolean limiter2Engaged;

  // Used to help us allow the angler to be touching a limit switch and move away from it. This value is only used for its sign.
  // This value is whatever value we have set the motor to, and we don't care what the ControlType is.
  private double setValue;

  public Angler() {

    anglerMotor = new CANSparkMax(AnglerConstants.ANGLER_SPARK_CAN_ID, MotorType.kBrushless);
    PIDController = anglerMotor.getPIDController();
    encoder = anglerMotor.getEncoder();
    anglerMotor.setClosedLoopRampRate(AnglerConstants.kRampRate);

    // stop the motor on enable
    anglerMotor.set(0.0);

    PIDController.setP(AnglerConstants.PIDFConstants[0][0]);
    PIDController.setI(AnglerConstants.PIDFConstants[0][1]);
    PIDController.setD(AnglerConstants.PIDFConstants[0][2]);
    PIDController.setIZone(AnglerConstants.PIDFConstants[0][3]);

    limitSwitch1 = new DigitalInput(AnglerConstants.limiter1Channel);
    limitSwitch2 = new DigitalInput(AnglerConstants.limiter2Channel);

    PIDLoopEngaged = false;
    limiter1Engaged = false;
    limiter2Engaged = false;
    setValue = 0;
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
    PIDController.setReference(encoder.getVelocity(), ControlType.kVelocity);
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
    if (!limiter1Engaged && !limiter2Engaged) {
      this.setValue = value;
      if (!PIDLoopEngaged) { this.reengagePIDLoop(); }
      PIDController.setReference(value, ControlType.kPosition);
    }
  }

  public void engageRawMotor(double percentOutput)
  {
    this.setValue = percentOutput;
    if (!limiter1Engaged && !limiter2Engaged && percentOutput >= -1 && percentOutput <= 1) {
      PIDLoopEngaged = false;
      anglerMotor.set(percentOutput);
    }
  }

  public void testPercentOutput() 
  {
    engageRawMotor(SmartDashboard.getNumber(AnglerConstants.kShuffleboardPercentName, 0.0));
    SmartDashboard.putNumber("AnglerRPMOut", encoder.getVelocity());
  }

  // Does the same thing as engageRawMotor(), except it ignores the limiter conditional so it can stop the motors no matter what
  public void limiterStopMotor()
  {
    PIDLoopEngaged = false;
    // Do NOT change the this.setValue variable for the limiter stopping the motor. It needs to remain where
    // it was so that we know which direction we were trying to go originally, and so that the motor doesn't
    // periodically stop and start over and over while the limit switch is held
    anglerMotor.set(0.0);
  }

  public void changeAngle(double angleChange)
  {
    double rotationsOfMotor = angleChange * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO;
    if ((encoder.getPosition() + rotationsOfMotor <= 80 * AnglerConstants.ROTATIONS_PER_DEGREE) &&
        (encoder.getPosition() + rotationsOfMotor >= 45 * AnglerConstants.ROTATIONS_PER_DEGREE)) {
      this.engagePIDMotorPosition(rotationsOfMotor);
    }
  }

  public void moveToAngle(double newAngle)
  {
    double oldAngleInRotationsOfMotor = encoder.getPosition();
    double differenceInRotations = (positionAt45Deg + (newAngle * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO)) - oldAngleInRotationsOfMotor;
    if ((newAngle <= 80) && (newAngle >= 45)) {
      this.engagePIDMotorPosition(differenceInRotations);
    }
  }


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
    positionAt45Deg = encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // When pressed, DigitalInput.get() returns FALSE!!! (makes total sense)
    if (!limitSwitch1.get() && setValue < 0) {
      limiterStopMotor();
      limiter1Engaged = true;
      limiter2Engaged = false;
      // System.out.println("LEFT LIMITER PRESSED ---------------");
    } else if (!limitSwitch2.get() && setValue > 0) {
      limiterStopMotor();
      limiter2Engaged = true;
      limiter1Engaged = false;
      // System.out.println("RIGHT LIMITER PRESSED ---------------");
    } else {
      limiter1Engaged = false;
      limiter2Engaged = false;
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Angler getInstance() {
    return m_Angler;
  }

}
