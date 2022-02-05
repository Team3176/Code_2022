// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.TransferConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Transfer extends SubsystemBase
{
  private static Transfer m_transfer = new Transfer();
  private CANSparkMax transferMotor;
  private SparkMaxPIDController pidController;
  private RelativeEncoder encoder;

  public Transfer()
  {
    transferMotor = new CANSparkMax(TransferConstants.TRANSFER_NEO1_CAN_ID, MotorType.kBrushless);
    pidController = transferMotor.getPIDController();
    encoder = transferMotor.getEncoder();
    
    transferMotor.setClosedLoopRampRate(TransferConstants.kRampRate);

    SmartDashboard.putNumber("percentTransfer", 0.0);
  }

  public void percentOutput() 
  {
    double output = SmartDashboard.getNumber("percentTransfer", 0.0);
    if (output >= -1 && output <= 1) { transferMotor.set(output); }
  }

  public void motor2Velocity(double velocity)
  {
    pidController.setReference(velocity, ControlType.kVelocity);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Transfer getInstance() {
    return m_transfer;
  }

}
