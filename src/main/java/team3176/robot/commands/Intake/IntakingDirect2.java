// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class IntakingDirect2 extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private CommandScheduler m_scheduler;

  public IntakingDirect2() {
    addRequirements(m_Intake, m_Indexer);
    m_scheduler = CommandScheduler.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_Indexer.setModeLoading();
    m_Indexer.Up();
    m_Intake.Extend();
    m_Intake.spinVelocityPercent(IntakeConstants.INTAKE_PCT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_Indexer.simpleIndexer();
    if(!m_Indexer.getSecondPos()) {
      m_Indexer.motorStop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
    // interrupted conditional is there so it doesn't screw up IntakeReject, which interrupts this command to reject balls
    if (!interrupted) {
      m_Intake.Retract();
    }
    */

    // check if IntakeReject is what interrupted this command. If it is NOT, then retract the intake.
    if (!m_scheduler.isScheduled(new IntakeReject())) {
      m_Intake.Retract();
    }

    // m_Indexer.setModeHolding();
    // m_Indexer.simpleIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
