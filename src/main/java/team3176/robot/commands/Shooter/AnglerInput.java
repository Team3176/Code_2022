// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Angler;

public class AnglerInput extends InstantCommand {
  private Angler m_Angler = Angler.getInstance();
  private double angle;

  public AnglerInput(double angle) {
    addRequirements(m_Angler);
    this.angle = angle;
  }

  @Override
  public void initialize() {
    m_Angler.moveToAngle(this.angle);
  }
}