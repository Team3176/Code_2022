// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ShooterLocationValues;
import team3176.robot.subsystems.feeder.Feeder;
import team3176.robot.subsystems.indexer.Indexer;
import team3176.robot.subsystems.intake.Intake;
import team3176.robot.subsystems.vision.Vision;

public class ShootVisionAutoFire extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double ty, tx;
  private double tv;

  public ShootVisionAutoFire() {
    addRequirements(m_Intake, m_Indexer, m_Feeder);
  }

  @Override
  public void initialize() {
    tx = m_Vision.tx.getDouble(0);
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getDouble(0);
  }

  @Override
  public void execute() {
    m_Vision.updateVisionData();
    tx = m_Vision.tx.getDouble(0);
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getDouble(0);
    
    if (tx < 3 && tx > -3 && tv == 1) {
      if(ty >= ShooterLocationValues.TY_2X_EDGE_OF_TARMAC ) {
        m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[0][0]);
        m_Indexer.setVelocity(ShooterLocationValues.POINTS[0][1]);
        m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[0][2]);
      }
      else if(ty >= ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE && ty < ShooterLocationValues.TY_2X_EDGE_OF_TARMAC) {
        m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[1][0]);
        m_Indexer.setVelocity(ShooterLocationValues.POINTS[1][1]);
        m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[1][2]);
      }
      else if(ty < ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE && ty >= ShooterLocationValues.TY_2X_LAUNCH_PAD) {
        m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[2][0]);
        m_Indexer.setVelocity(ShooterLocationValues.POINTS[2][1]);
        m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[2][2]);
      }
      else if(ty < ShooterLocationValues.TY_2X_LAUNCH_PAD && ty >= ShooterLocationValues.TY_2X_WALL_ZONE) {
        m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[3][0]);
        m_Indexer.setVelocity(ShooterLocationValues.POINTS[3][1]);
        //m_Indexer.Up();
        m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[3][2]);
      }
      else if(ty < ShooterLocationValues.TY_2X_WALL_ZONE) {
        m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[4][0]);
        m_Indexer.setVelocity(ShooterLocationValues.POINTS[4][1]);
        m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[4][2]);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.stopMotor();
    m_Indexer.motorStop();
    m_Feeder.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
