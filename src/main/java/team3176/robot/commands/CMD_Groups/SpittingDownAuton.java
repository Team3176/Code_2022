// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.feeder.Feeder;
import team3176.robot.subsystems.flywheel.Flywheel;
import team3176.robot.subsystems.indexer.Indexer;
import team3176.robot.subsystems.intake.Intake;

public class SpittingDownAuton extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();
  private Flywheel m_Flywheel = Flywheel.getInstance();
  public SpittingDownAuton() {
    addRequirements(m_Intake, m_Indexer, m_Feeder, m_Flywheel);
  }

  @Override
  public void initialize() { //TODO: BETTER PCTs
    m_Intake.Extend();
    // Timer.delay(0.5);
    m_Intake.spinVelocityPercent(-0.5);
    m_Indexer.setPCT(-0.5);
    m_Feeder.setPCT(-0.5);
    m_Flywheel.setPCT(-0.2, -0.2);
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