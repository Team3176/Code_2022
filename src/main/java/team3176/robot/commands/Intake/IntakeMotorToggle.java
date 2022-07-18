// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.intake.Intake;

public class IntakeMotorToggle extends InstantCommand {
  private Intake m_Intake = Intake.getInstance();
  private boolean state;
  public IntakeMotorToggle() {
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {
    state = m_Intake.getMotorSetting();
    if(state) {m_Intake.stopMotor();}
    else {m_Intake.spinVelocityPercent(IntakeConstants.INTAKE_PCT);}
  }
}
