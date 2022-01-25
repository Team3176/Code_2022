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

  public Angler() {

    anglerMotor = new CANSparkMax(AnglerConstants.ANGLER_SPARK_CAN_ID, MotorType.kBrushless);
    PIDController = anglerMotor.getPIDController();
    encoder = anglerMotor.getEncoder();
    anglerMotor.setClosedLoopRampRate(AnglerConstants.kRampRate);

    ticsPerRevolution = encoder.getCountsPerRevolution();

    limitSwitch1 = new DigitalInput(AnglerConstants.limiter1Channel);
    limitSwitch2 = new DigitalInput(AnglerConstants.limiter2Channel);

  }

  public void changeAngle(double angleChange)
  {
    double rotationsOfMotor = angleChange * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO;
    if ((encoder.getPosition() + rotationsOfMotor <= 80 * AnglerConstants.ROTATIONS_PER_DEGREE) &&
        (encoder.getPosition() + rotationsOfMotor >= 45 * AnglerConstants.ROTATIONS_PER_DEGREE)) {
      PIDController.setReference(rotationsOfMotor, ControlType.kPosition);
    }
  }

  public void moveToAngle(double newAngle)
  {
    double oldAngleInRotationsOfMotor = encoder.getPosition();
    double differenceInRotations = (newAngle * AnglerConstants.ROTATIONS_PER_DEGREE * AnglerConstants.ANGLER_GEAR_RATIO) - oldAngleInRotationsOfMotor;
    if ((newAngle <= 80) && (newAngle >= 45)) {
      PIDController.setReference(differenceInRotations, ControlType.kPosition);
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
  */



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
