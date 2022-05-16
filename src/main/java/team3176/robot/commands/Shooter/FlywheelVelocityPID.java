// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Flywheel;

public class FlywheelVelocityPID extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  double pct1 = 0.30;
  double pct2 = 0.30;

  public FlywheelVelocityPID() {
    addRequirements(m_Flywheel);
  }

  public FlywheelVelocityPID(double pct1, double pct2) {
    addRequirements(m_Flywheel);
    this.pct1 = pct1;
    this.pct2 = pct2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.spinMotorsVelocityPID(this.pct1, this.pct2);
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
