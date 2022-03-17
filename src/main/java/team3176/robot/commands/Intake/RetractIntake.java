// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class RetractIntake extends CommandBase {
  private Intake m_Intake = Intake.getInstance();
  public RetractIntake() {
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_Intake.getPistonSetting()) {
      m_Intake.Retract();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
