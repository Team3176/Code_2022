// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Flywheel;

public class DecrementFlywheelPID1 extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  double pct1 = 0.30;
  double pct2 = 0.30;
  double pct1b = 0;

  public DecrementFlywheelPID1() {
    addRequirements(m_Flywheel);
    this.pct1 = m_Flywheel.flywheel1Pct;
    this.pct2 = m_Flywheel.flywheel2Pct;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheel.decrFlywheelVelocity1();
    this.pct1b = m_Flywheel.flywheel1Pct;
    m_Flywheel.spinMotorsVelocityPID(this.pct1b, this.pct2);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (this.pct1b < this.pct1) {
      return true;
    } else {
    return false;
    }
  }
}
