// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Indexer;

public class IndexerHoldingMode extends InstantCommand {
  private Indexer m_Indexer = Indexer.getInstance();
  public IndexerHoldingMode() {}

  @Override
  public void initialize() {m_Indexer.setModeHolding();}
}
