// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import team3176.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
    private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID1_FWD_CHAN, IntakeConstants.DSOLENOID1_REV_CHAN);
    private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID2_FWD_CHAN, IntakeConstants.DSOLENOID2_REV_CHAN);
    private TalonSRX intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
    private boolean pistonSetting = false;
    private boolean isSmartDashboardTestControlsShown;
    public String mode = "";

    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();
    private static Intake instance;

  private Intake(IntakeIO io) 
  {
    this.io = io;
  }

  public void Extend() 
  {
    pistonSetting = true;
    piston1.set(Value.kForward);
    piston2.set(Value.kForward);
  }

  public void Retract() 
  {
    pistonSetting = false;
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);
  }

  public void spinVelocityPercent(double pct)
  {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, pct);
  }

  public void stopMotor()
  {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public boolean getPistonSetting()
  {
    return pistonSetting;
  }

  public static Intake getInstance() {
    if(instance == null) {instance = new Intake(new IntakeIO() {});}
    return instance;
  }

    public void putSmartDashboardControlCommands() 
    {
     SmartDashboard.putNumber("Intake Falcon PCT", 0);
     SmartDashboard.putBoolean("Intake Piston Setting", true);
     isSmartDashboardTestControlsShown = true;
    }

   public void setValuesFromSmartDashboard() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, SmartDashboard.getNumber("Intake Falcon PCT", 0));
    pistonSetting = SmartDashboard.getBoolean("Intake Piston Setting", true);
    if (pistonSetting)
    {
      Retract();
    }
    else 
    {
      Extend();
    }
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Intake", inputs);
    // Logger.getInstance().recordOutput("Intake/Velocity", getIntakeVelocity());
    // Logger.getInstance().recordOutput("Intake/PistonState", getPistonState());
    // if(!isSmartDashboardTestControlsShown) {
    //   if(mode.equals("test")) setValuesFromSmartDashboard();
    // }
    // putSmartDashboardControlCommands();
    if(mode.equals("test")) {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }
  }

  public void runVoltage(double volts) 
  {
    io.setVoltage(volts);
  }

  public void setVelocity(double intakeVelocity)
  {
    io.setVelocity(intakeVelocity);
  }

  public void setPiston(boolean isExtend)
  {
    io.setPiston(isExtend);
  }

  public double getIntakeVelocity() 
  {
    return inputs.velocity;
  }

  public boolean getPistonState()
  {
    return inputs.isExtend;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
