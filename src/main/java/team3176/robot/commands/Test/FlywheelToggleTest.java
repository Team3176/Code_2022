// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Flywheel;

public class FlywheelToggleTest extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  public FlywheelToggleTest() {
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.putSmartDashboardControlCommands();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Flywheel.setValuesFromSmartDashboard();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopWithPCT();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
