// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.indexer.Indexer;

public class IndexerEnterAndSit extends CommandBase {
  /** Creates a new IndexerEnterAndSit. */
  private Indexer m_Indexer = Indexer.getInstance();

  public IndexerEnterAndSit() {
  addRequirements(m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_Indexer.isFirstPos() && m_Indexer.isSecondPos() && !m_Indexer.isThirdPos())
    {
      while (!m_Indexer.isThirdPos())
      {
        m_Indexer.IndexerSpin();
      }
      m_Indexer.motorStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
