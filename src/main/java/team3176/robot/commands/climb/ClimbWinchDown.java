// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.MasterConstants;
import team3176.robot.subsystems.Climb;

/**
 * Winches to the Lowest Position
 */

public class ClimbWinchDown extends InstantCommand {
  Climb m_Climb = Climb.getInstance();

  public ClimbWinchDown() {
    addRequirements(m_Climb);
  }

  @Override
  public void initialize() {
    if(!MasterConstants.ISCLIMBPASSIVE) {m_Climb.winchDown();}
  }
}
