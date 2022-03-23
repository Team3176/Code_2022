// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ShooterLocationValues;
import team3176.robot.subsystems.Feeder;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;
import team3176.robot.subsystems.Vision;

public class ShootVisionAuton extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double ty;
  private boolean tv;

  public ShootVisionAuton() {
    addRequirements(m_Intake, m_Indexer, m_Feeder);
  }

  @Override
  public void initialize() {
    m_Vision.updateVisionData();
    ty = m_Vision.ty.getDouble(0);
    tv = m_Vision.tv.getBoolean(false);

    if(ty >= ShooterLocationValues.TY_2X_EDGE_OF_TARMAC || (ty == 0 && !tv)) {
      System.out.println("TARMAC ZONE");
      m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[0][0]);
      m_Indexer.setVelocity(ShooterLocationValues.POINTS[0][1]);
      m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[0][2]);
    }
    else if(ty >= ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE && ty < ShooterLocationValues.TY_2X_EDGE_OF_TARMAC) {
      System.out.println("TARMAC LINE");
      m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[1][0]);
      m_Indexer.setVelocity(ShooterLocationValues.POINTS[1][1]);
      m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[1][2]);
    }
    else if(ty < ShooterLocationValues.TY_2X_MID_OF_TARMAC_LINE && ty >= ShooterLocationValues.TY_2X_LAUNCH_PAD) {
      System.out.println("LAUNCH PAD ZONE");
      m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[2][0]);
      m_Indexer.setVelocity(ShooterLocationValues.POINTS[2][1]);
      m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[2][2]);
    }
    else if(ty < ShooterLocationValues.TY_2X_LAUNCH_PAD && ty >= ShooterLocationValues.TY_2X_WALL_ZONE) {
      System.out.println("PRE-WALL ZONE");
      m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[3][0]);
      m_Indexer.setVelocity(ShooterLocationValues.POINTS[3][1]);
      m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[3][2]);
    }
    else if(ty < ShooterLocationValues.TY_2X_WALL_ZONE) {
      System.out.println("WALL ZONE");
      m_Intake.spinVelocityPercent(ShooterLocationValues.POINTS[4][0]);
      m_Indexer.setVelocity(ShooterLocationValues.POINTS[4][1]);
      m_Feeder.setVelocityPID(ShooterLocationValues.POINTS[4][2]);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
