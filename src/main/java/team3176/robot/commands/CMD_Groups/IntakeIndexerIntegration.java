// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.constants.MasterConstants;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIndexerIntegration extends CommandBase {
  private Indexer m_Indexer = Indexer.getInstance();
  private Intake m_Intake = Intake.getInstance();
  private Timer timeElasped = new Timer();
  private int numTimes100 = 0;
  private int lastState = 111;
  private int currState = 111;
  // private double timeLimit = 10;

  public IntakeIndexerIntegration() {
    addRequirements(m_Indexer, m_Intake);
    // if(Timer.getMatchTime() > (MasterConstants.FULL_MATCH_TIME - 15)) {timeLimit = 4;}
  }

  @Override
  public void initialize() {
    timeElasped.start();
    m_Intake.Extend();
    m_Intake.spinVelocityPercent(IntakeConstants.INTAKE_PCT);
    m_Indexer.Up();
  }

  @Override
  public void execute() {
    lastState = currState; 
    currState = m_Indexer.reportState();
    if(currState != lastState) {
      
    }
    if (currState != lastState && currState == 100) {
      numTimes100++;
      m_Indexer.requestState(010);
    }
    if (currState != lastState && currState == 110)
      numTimes100++;
  }

  @Override
  public void end(boolean interrupted) {
    if (numTimes100 == 1) {
      m_Indexer.requestState(100);
    }
    m_Intake.Retract();
    m_Intake.stopMotor();
  }

  @Override
  public boolean isFinished() {
    if (numTimes100 == 2) {return true;}
    // else if (timeElasped.get() == timeLimit) {return true;} //TODO: LOWER TIME
    return false;
  }
}
