// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ShooterLocationValues;
import team3176.robot.subsystems.Angler;
import team3176.robot.subsystems.Flywheel;
import team3176.robot.subsystems.Vision;

public class FlywheelAngleVision extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Angler m_Angler = Angler.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double ty;
  private boolean tv;
  public FlywheelAngleVision() {
    addRequirements(m_Flywheel, m_Angler);
  }

  @Override
  public void initialize() {
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getBoolean(false);
  }

  @Override
  public void execute() {
    m_Vision.updateVisionData();
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getBoolean(false);

    if(!tv) {
      m_Angler.moveToAngle(ShooterLocationValues.POINTS[1][5]);
    }
    else if(ty >= ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE) {
      m_Angler.moveToAngle(ShooterLocationValues.POINTS[1][5]);
      m_Flywheel.spinMotorsVelocityPID(ShooterLocationValues.POINTS[1][3], ShooterLocationValues.POINTS[1][4]);
    }
    else if(ty < ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE) {
      m_Angler.moveToAngle(ShooterLocationValues.POINTS[2][5]);
      m_Flywheel.spinMotorsVelocityPID(ShooterLocationValues.POINTS[2][3], ShooterLocationValues.POINTS[2][4]);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
    // m_Angler.moveToAngle(52);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
