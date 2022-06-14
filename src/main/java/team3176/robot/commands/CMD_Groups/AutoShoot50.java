// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.feeder.Feeder;
import team3176.robot.subsystems.indexer.Indexer;
import team3176.robot.subsystems.intake.Intake;

public class AutoShoot50 extends InstantCommand {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();

  public AutoShoot50() {
    addRequirements(m_Intake, m_Indexer, m_Feeder);
  }

  @Override
  public void initialize() {
    m_Intake.spinVelocityPercent(0.5);
    m_Indexer.setVelocity(0.5);
    m_Feeder.setVelocityPID(0.5);
  }
}
