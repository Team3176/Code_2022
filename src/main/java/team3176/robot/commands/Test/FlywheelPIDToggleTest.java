// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.flywheel.Flywheel;

public class FlywheelPIDToggleTest extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private double one, two;
  public FlywheelPIDToggleTest() {
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    one = m_Flywheel.getStartPercent1();
    two = m_Flywheel.getStartPercent2();
    m_Flywheel.putSmartDashboardPIDControlCommands(one, two);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Flywheel.setPIDValuesFromSmartDashboard();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
