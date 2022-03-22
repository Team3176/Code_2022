// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Angler;
import team3176.robot.subsystems.Climb;

/* Extend the Pistons */

public class ClimbPistonEngage extends InstantCommand{
  private Climb m_Climb = Climb.getInstance();
  private Angler m_Angler = Angler.getInstance();

  public ClimbPistonEngage() {
    addRequirements(m_Climb, m_Angler);
  }

  @Override
  public void initialize() {
    m_Angler.moveToAngle(90);
    m_Climb.climbPistonsEngage();
  }
}
