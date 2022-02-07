// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import team3176.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private static Intake instance = new Intake();
    private DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.DSOLENOID1_FWD_CHAN, IntakeConstants.DSOLENOID1_REV_CHAN);
    private DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.DSOLENOID2_FWD_CHAN, IntakeConstants.DSOLENOID2_REV_CHAN);
    private TalonFX intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
    private boolean pistonSetting = false;
    

  public Intake() {
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
