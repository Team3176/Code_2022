// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Flywheel;

public class FlywheelBackspinVelocityPID extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  public FlywheelBackspinVelocityPID() {
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.spinMotorsVelocityPID(0.45, 0.15);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
