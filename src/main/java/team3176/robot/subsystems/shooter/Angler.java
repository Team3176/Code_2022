// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import team3176.robot.constants.AnglerConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Angler extends SubsystemBase {
  
  private CANSparkMax anglerMotor;
  private SparkMaxPIDController PIDController;
  private RelativeEncoder encoder;

  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;

  private int ticsPerRevolution;
  private double positionAt45Deg;

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

    ticsPerRevolution = encoder.getCountsPerRevolution();

    limitSwitch1 = new DigitalInput(AnglerConstants.limiter1Channel);
    limitSwitch2 = new DigitalInput(AnglerConstants.limiter2Channel);

    PIDLoopEngaged = true;
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
    PIDController.setReference(encoder.getVelocity(), ControlType.kVelocity);
  }

  /**
   * Should be called anytime the motor should be set in this subsystem instead of writing the set line. This method accounts for the possibility of the
   * PID loop being disengaged from setting the raw speed using the simple set() method and the limiter being engaged.
   * @param value
   * @param controlType
   * @author Jared Brown
   */
  public void engagePIDMotor(double value, ControlType controlType)
  {
    this.setValue = value;
    if (!PIDLoopEngaged) { this.reengagePIDLoop(); }
    PIDController.setReference(value, controlType);
  }

  public void engageRawMotor(double velocity)
  {
    PIDLoopEngaged = false;
    anglerMotor.set(velocity);
  }

  public void changeAngle(double angleChange)
  {
    double rotationsOfMotor = angleChange * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO;
    if ((encoder.getPosition() + rotationsOfMotor <= 80 * AnglerConstants.ROTATIONS_PER_DEGREE) &&
        (encoder.getPosition() + rotationsOfMotor >= 45 * AnglerConstants.ROTATIONS_PER_DEGREE)) {
      this.engagePIDMotor(rotationsOfMotor, ControlType.kPosition);
    }
  }

  public void moveToAngle(double newAngle)
  {
    double oldAngleInRotationsOfMotor = encoder.getPosition();
    double differenceInRotations = (newAngle * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO) - oldAngleInRotationsOfMotor;
    if ((newAngle <= 80) && (newAngle >= 45)) {
      this.engagePIDMotor(differenceInRotations, ControlType.kPosition);
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
    PIDController.setReference(rotationsPerMin, ControlType.kVelocity);
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

    if ((limitSwitch1.get() && setValue < 0) || (limitSwitch2.get() && setValue > 0)) {
      engageRawMotor(0.0);
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
