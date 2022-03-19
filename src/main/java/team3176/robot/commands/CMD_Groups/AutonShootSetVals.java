// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Feeder;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;

public class AutonShootSetVals extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();

  public AutonShootSetVals() {
    addRequirements(m_Intake, m_Indexer, m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.spinVelocityPercent(0.5);
    m_Indexer.setVelocity(0.45);
    m_Feeder.setVelocityPID(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}