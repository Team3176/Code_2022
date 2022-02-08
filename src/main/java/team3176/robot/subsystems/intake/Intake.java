// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.IntakeConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
    private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_CAN_ID);
    private static Intake instance = new Intake();
    private boolean pistonSetting = false;
    

  public Intake() {

    SmartDashboard.putNumber("percentIntake", 0.0);

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
    intakeMotor.set(TalonFXControlMode.PercentOutput, pct);
  }

  public void stopMotor()
  {
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public boolean getPistonSetting()
  {
    return pistonSetting;
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber("percentIntake", 0.0);
    if (output >= -1 && output <= 1) { intakeMotor.set(TalonFXControlMode.PercentOutput, output); }
  }

  public static Intake getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
