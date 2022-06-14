// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.intake.Intake;

public class IntakePistonToggle extends InstantCommand {
  private Intake m_Intake = Intake.getInstance();
  private boolean state;
  public IntakePistonToggle() {
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {
    state = m_Intake.getPistonSetting();
    if(state) {m_Intake.Retract();}
    else {m_Intake.Extend();}
  }
}
