// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.Intake;

public class IntakeReject extends CommandBase {
  /** Creates a new IntakeReject. */
  private Intake m_intake = Intake.getInstance();
  private double startTime;

  public IntakeReject() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if (!m_intake.isExtended()) { m_intake.Extend(); }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.spinVelocityPercent(-IntakeConstants.INTAKE_PCT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor();
    m_intake.Retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startTime) >= IntakeConstants.kTimeForRejection);
  }
}
