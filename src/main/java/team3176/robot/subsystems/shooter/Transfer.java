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
import team3176.robot.constants.TransferConstants;

public class Transfer extends SubsystemBase {

  private CANSparkMax transferMotor1;
  private CANSparkMax transferMotor2;

  public Transfer() {

    transferMotor1 = new CANSparkMax(TransferConstants.TRANSFER_NEO1_CAN_ID, MotorType.kBrushless);
    transferMotor2 = new CANSparkMax(TransferConstants.TRANSFER_NEO2_CAN_ID, MotorType.kBrushless);

  }

  public void motor1Stop() {




  }

  public void motor1FullPow() {



    
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
