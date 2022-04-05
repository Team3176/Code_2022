// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class IntakeExtendReverse extends CommandBase {
  private Intake m_Intake = Intake.getInstance();

  public IntakeExtendReverse() {
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!(m_Intake.getPistonSetting())) {
      m_Intake.Extend();
      m_Intake.spinVelocityPercent(-0.5); // <-- temporary value //TODO: FIND ACTUAL VALUE
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true; //TODO: FIX LOGIC
  }
}