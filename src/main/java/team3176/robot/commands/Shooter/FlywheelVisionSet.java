// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.flywheel.Flywheel;
import team3176.robot.subsystems.vision.Vision;

public class FlywheelVisionSet extends InstantCommand {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Vision m_Vision = Vision.getInstance();

  public FlywheelVisionSet() {
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    int visionTicksPer100MS = 100; //m_Vision.getBestFlywheelTicksPer100MS(); //TODO: ADD VISION
    m_Flywheel.spinMotors(visionTicksPer100MS);
  }
}
