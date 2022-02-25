// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.shooter.Angler;

public class AnglerMin extends InstantCommand {
  private Angler m_Angler = Angler.getInstance();

  public AnglerMin() {
    addRequirements(m_Angler);
  }

  @Override
  public void initialize() {
    m_Angler.moveToAngle(45.0);
  }
}