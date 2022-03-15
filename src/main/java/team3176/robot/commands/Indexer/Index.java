// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Indexer;

public class Index extends CommandBase {
  private Indexer m_Indexer = Indexer.getInstance();
  public Index() {
    addRequirements(m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Indexer.setModeHolding();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Indexer.index();
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