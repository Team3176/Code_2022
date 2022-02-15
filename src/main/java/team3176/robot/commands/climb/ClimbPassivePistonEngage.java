// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.climb.Climb;

/**
 * Extend Primary Pistons
 */

public class ClimbPassivePistonEngage extends InstantCommand{
  private Climb m_Climb = Climb.getInstance();

  public ClimbPassivePistonEngage() {
    addRequirements(m_Climb);
  }

  @Override
  public void initialize() {
    m_Climb.passivePistonsEngage();
  }
}
