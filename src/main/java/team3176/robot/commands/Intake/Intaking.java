// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.indexer.Indexer;
import team3176.robot.subsystems.intake.Intake;

public class Intaking extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  public Intaking() {
    addRequirements(m_Intake, m_Indexer);
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
    m_Indexer.simpleIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.Retract();
    Timer.delay(1); //TODO: TUNE AND FIND ALTERNATIVE
    m_Intake.stopMotor();
    // m_Indexer.setModeHolding();
    m_Indexer.simpleIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
