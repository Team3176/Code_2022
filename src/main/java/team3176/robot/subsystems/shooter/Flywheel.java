// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.FlywheelConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import team3176.robot.subsystems.shooter.FlywheelIO.FlywheelIOInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends SubsystemBase {

  private TalonFX flywheelMotor1;
  private TalonFX flywheelMotor2;

  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private static Flywheel instance;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = "";

  public Flywheel(FlywheelIO io)
  {
    this.io = io;

    flywheelMotor1 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON1_CAN_ID);
    flywheelMotor2 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON2_CAN_ID);

    flywheelMotor1.configFactoryDefault();
    flywheelMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor1.setSensorPhase(true);
    flywheelMotor1.configClosedloopRamp(FlywheelConstants.kRampRate, FlywheelConstants.kTimeoutMS);

    flywheelMotor2.configFactoryDefault();
    flywheelMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    flywheelMotor2.configAllowableClosedloopError(0, FlywheelConstants.kPIDLoopIndex, FlywheelConstants.kTimeoutMS);
    // This will (hopefully) invert the second motor
    flywheelMotor2.setSensorPhase(false);
    flywheelMotor2.configClosedloopRamp(FlywheelConstants.kRampRate, FlywheelConstants.kTimeoutMS);
  }

  public void spinMotors(double ticksPer100ms)
  {
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }

  /*
  public void spinMotors2(double metersPerSecond)
  {
    // double ticsPer100ms = --MATH!--
    flywheelMotor1.set(TalonFXControlMode.Velocity, ticksPer100ms);
    flywheelMotor2.set(TalonFXControlMode.Velocity, ticksPer100ms);
  }
  */

  public void percentOutput_1() 
  {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor1.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly1Tics/100msOut", flywheelMotor1.getSelectedSensorVelocity());
  }

  public void percentOutput_2() 
  {
    double output = SmartDashboard.getNumber(FlywheelConstants.kShuffleboardPercentName2, 0.0);
    if (output >= -1 && output <= 1) { flywheelMotor2.set(ControlMode.PercentOutput, output); }
    SmartDashboard.putNumber("Fly2Tics/100msOut", flywheelMotor2.getSelectedSensorVelocity());
  }

  public void stopMotors()
  {
    flywheelMotor1.set(TalonFXControlMode.PercentOutput, 0.0);
    flywheelMotor2.set(TalonFXControlMode.PercentOutput, 0.0);
  }

    public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Flywheel 1 PCT", 0);
    SmartDashboard.putNumber("Flywheel 2 PCT", 0);
    isSmartDashboardTestControlsShown = true;
    }

    public void setValuesFromSmartDashboard() 
    {
      flywheelMotor1.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Flywheel 1 PCT", 0));
      flywheelMotor2.set(TalonFXControlMode.PercentOutput, SmartDashboard.getNumber("Flywheel 2 PCT", 0));
    }

  @Override
  public void periodic() {
    if(mode.equals("test"))
    {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Flywheel getInstance() {
    if(instance == null) {instance = new Flywheel(new FlywheelIO() {});}
    return instance;
  }
  
}
