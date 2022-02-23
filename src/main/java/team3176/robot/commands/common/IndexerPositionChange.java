// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.indexer.Indexer;

public class IndexerPositionChange extends CommandBase {
  private Indexer m_Indexer = Indexer.getInstance();
  private int wantedPos;
  private int startingPos;

  public IndexerPositionChange(int wantedPos) {
    addRequirements(m_Indexer);
    this.wantedPos = wantedPos;
  }

  @Override
  public void initialize() {
    startingPos = m_Indexer.reportState();
    if(startingPos != wantedPos) {
      m_Indexer.requestState(wantedPos);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Indexer.motorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (wantedPos == m_Indexer.reportState());
  }
}
