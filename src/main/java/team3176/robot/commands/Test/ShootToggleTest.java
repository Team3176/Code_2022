// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Feeder;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;

public class ShootToggleTest extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  private Feeder m_Feeder = Feeder.getInstance();
  public ShootToggleTest() {
    addRequirements(m_Intake, m_Indexer, m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.putSmartDashboardControlCommands();
    m_Indexer.putSmartDashboardControlCommands();
    m_Feeder.putSmartDashboardControlCommands();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setValuesFromSmartDashboard();
    m_Indexer.setValuesFromSmartDashboard();
    m_Feeder.setValuesFromSmartDashboard();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stopMotor();
    m_Indexer.motorStop();
    m_Feeder.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
