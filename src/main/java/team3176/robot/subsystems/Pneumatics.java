// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private static Pneumatics instance = new Pneumatics();
  public static Pneumatics getInstance() {return instance;}

  private Compressor m_Compressor;
  private PneumaticHub m_Hub;

  public Pneumatics() {
    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    m_Hub = new PneumaticHub();
    // TODO: ADD A WAY TO CLEAR STICKY FAULTS
    // m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    m_Compressor.enableDigital();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PSI", m_Compressor.getPressure());
    SmartDashboard.putBoolean("Climb", m_Compressor.getPressure() > 40); //Whatever PSI Climb is
    SmartDashboard.putBoolean("High Climb", m_Compressor.getPressure() > 60); //Whatever PSI High Climb is
  }

  public void disableCompressor() {m_Compressor.disable();}
  public void enableCompressor() {m_Compressor.enableDigital();}
}
