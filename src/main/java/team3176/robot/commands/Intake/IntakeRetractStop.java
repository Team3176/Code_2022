// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Intake;

public class IntakeRetractStop extends InstantCommand {
  private Intake m_Intake = Intake.getInstance();
  public IntakeRetractStop() {
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.Retract();
    m_Intake.stopMotor();
  }
}
