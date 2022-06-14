// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.flywheel.Flywheel;

public class FlywheelPCT extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  public FlywheelPCT() {
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.setPCT(0.2, 0.2);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopWithPCT();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
