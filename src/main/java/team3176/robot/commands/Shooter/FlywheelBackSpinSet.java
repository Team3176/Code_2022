// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Flywheel;
import team3176.robot.subsystems.Vision;

public class FlywheelBackSpinSet extends InstantCommand {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private double front;
  private double back;

  public FlywheelBackSpinSet(double front, double back) {
    this.front = front;
    this.back = back;
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    m_Flywheel.spinMotorsVelocityPID(this.front, this.back);
  }
}