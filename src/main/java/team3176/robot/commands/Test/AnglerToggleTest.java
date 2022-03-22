// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Angler;

public class AnglerToggleTest extends CommandBase {
  Angler m_Angler = Angler.getInstance();
  double anglerS;
  public AnglerToggleTest() {
    addRequirements(m_Angler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    anglerS = m_Angler.getStartAngle();
    m_Angler.putSmartDashboardControlCommands(anglerS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Angler.setValuesFromSmartDashboard();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Angler.moveToAngle(90);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
