// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.angler.Angler;
import team3176.robot.subsystems.flywheel.Flywheel;
import team3176.robot.subsystems.vision.Vision;

public class FlywheelAngleVisionIntAutoFire extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Angler m_Angler = Angler.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double ty;
  private boolean tv;
  public FlywheelAngleVisionIntAutoFire() {
    addRequirements(m_Flywheel, m_Angler);
  }

  @Override
  public void initialize() {
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getBoolean(false);    
    m_Vision.setVisionSpinCorrection(true);

  }

  @Override
  public void execute() {
    m_Vision.updateVisionData();
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getBoolean(false);

    m_Angler.moveToAngle(65);
    m_Flywheel.spinMotorsVelocityPID(thirdPowInt() * 0.95, 0.20);
  }

  public double secondPowInt() {
    // y = 0.0013x^2 - 0.0041x + 0.3217
    double pct = (0.0013 * ty * ty) - (0.0041 * ty) + (0.3217);
    return pct;
  }
  public double thirdPowInt() {
    // y = 0.0007x^3 + 0.0059x^2 - 0.0046x + 0.3129
    // y = 0.0003x^3 + 0.0034x^2 - 0.0043x + 0.3177
    // double pct = (0.0003 * ty * ty * ty) + (0.0034 * ty * ty) - (0.0043 * ty) + (0.3177);
    // double pct = (0.00009 * ty * ty * ty) + (0.0014 * ty * ty) - (0.0056 * ty) + (0.3221);
    double pct = (0.00003 * ty * ty * ty) + (0.0006 * ty * ty) - (0.008 * ty) + (0.3225);
    return pct;
  }

  @Override
  public void end(boolean interrupted) { 
    m_Vision.setVisionSpinCorrection(false);
  }    


  @Override
  public boolean isFinished() {
    return false;
  }
}