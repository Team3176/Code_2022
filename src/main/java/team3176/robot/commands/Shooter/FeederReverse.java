// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.TransferConstants;
import team3176.robot.subsystems.Feeder;

public class FeederReverse extends InstantCommand {
  private Feeder m_Feeder = Feeder.getInstance();

  public FeederReverse() {
    addRequirements(m_Feeder);
  }

  @Override
  public void initialize() {
    m_Feeder.motor2Velocity(-TransferConstants.MAX_VELOCITY);
  }
}
