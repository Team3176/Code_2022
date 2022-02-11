// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.lib.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.climb.ClimbActive;

/**
 * Winches to the Longest Position
 */

public class ClimbWinchUp extends InstantCommand{
  ClimbActive m_Climb = ClimbActive.getInstance();

  public ClimbWinchUp() {
    addRequirements(m_Climb);
  }

  @Override
  public void initialize() {
    m_Climb.winchUp();
  }
}
