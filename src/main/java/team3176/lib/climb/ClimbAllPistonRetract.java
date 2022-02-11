// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.lib.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.climb.ClimbActive;

/**
 * Retracts all of the Climb Pistons at Once
 */

 public class ClimbAllPistonRetract extends InstantCommand{
  private ClimbActive m_Climb = ClimbActive.getInstance();

  public ClimbAllPistonRetract() {
    addRequirements(m_Climb);
  }

  @Override
  public void initialize() {
    m_Climb.primaryPistonsRetract();
    m_Climb.secondaryPistonsRetract();
  }
}