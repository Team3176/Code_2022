// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Vision;

public class VisionZoom1x extends InstantCommand {
  private Vision m_Vision = Vision.getInstance();
  public VisionZoom1x() {}

  @Override
  public void initialize() {
    m_Vision.setActivePipeline(1);
  }
}
