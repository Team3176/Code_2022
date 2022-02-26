// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.shooter.Feeder;

public class FeederStop extends InstantCommand {
  private Feeder m_Feeder = Feeder.getInstance();

  public FeederStop() {
    addRequirements(m_Feeder);
  }

  @Override
  public void initialize() {
    m_Feeder.stopMotor();
  }
}
