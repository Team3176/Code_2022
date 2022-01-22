// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import team3176.robot.constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {

  private Flywheel m_flywheel = new Flywheel();
  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  public Flywheel()
  {
    flywheelMotor1 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON1_CAN_ID);
    flywheelMotor2 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON2_CAN_ID);

    flywheelMotor1.configFactoryDefault();
    flywheelMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.setSensorPhase(true);

    flywheelMotor2.configFactoryDefault();
    flywheelMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor2.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    // This will (hopefully) invert the second motor
    flywheelMotor2.setSensorPhase(false);
  }

  public void spinMotors(double ticksPer100ms)
  {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  public void spinMotors2(double metersPerSecond)
  {
    
  }

  public void stopMotors()
  {
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Flywheel getInstance() {
    return m_flywheel;
  }
  
}
