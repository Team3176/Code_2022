// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Flywheel;

public class IncrementFlywheelPID2 extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  double pct1 = 0.30;
  double pct2 = 0.30;
  double pct2b = 0;

  public IncrementFlywheelPID2() {
    addRequirements(m_Flywheel);
    this.pct1 = m_Flywheel.flywheel1Pct;
    this.pct2 = m_Flywheel.flywheel2Pct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.incrFlywheelVelocity2();
    this.pct2b = m_Flywheel.flywheel2Pct;
    m_Flywheel.spinMotorsVelocityPID(this.pct1, this.pct2b);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (this.pct2b > this.pct2) {
      return true;
    } else {
    return false;
    }
  }
}
