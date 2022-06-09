// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;

public class IntakingDirectSpacing extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private int ballCount;
  private boolean intakeLastLineState;

  public IntakingDirectSpacing() { //TODO: Globalize Ball Count
    addRequirements(m_Intake, m_Indexer);
  }

  @Override
  public void initialize() {
    ballCount = 0;
    intakeLastLineState = m_Intake.getLine();
    m_Indexer.Up();
    m_Intake.Extend();
    m_Intake.spinVelocityPercent(IntakeConstants.INTAKE_PCT);
  }

  @Override
  public void execute() {
    if(!m_Intake.getLine() && (m_Intake.getLine() != intakeLastLineState)) {ballCount++;}
    if(!m_Indexer.getFirstPos() && ballCount == 1) {
      m_Indexer.motorStop();
    }
    if(!m_Indexer.getFirstPos() && ballCount == 2) {
      while(m_Indexer.getThirdPos() || m_Indexer.getFirstPos()) {
        m_Indexer.Up();
      }
    }

    intakeLastLineState = m_Intake.getLine();
  }

  @Override
  public void end(boolean interrupted) {
    m_Intake.Retract();
    m_Indexer.motorStop();
    m_Intake.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return (!m_Indexer.getThirdPos() && !m_Indexer.getFirstPos());
  }
}