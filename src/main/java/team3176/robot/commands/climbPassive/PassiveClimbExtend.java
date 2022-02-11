// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.climbPassive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.climb.ClimbPassive;

/**
 * Retracts the Pistons on the Passive Climb
 */

public class PassiveClimbExtend extends InstantCommand {
  private ClimbPassive m_Climb = ClimbPassive.getInstance();

  public PassiveClimbExtend() {
    addRequirements(m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climb.extendPistons();
  }
}
