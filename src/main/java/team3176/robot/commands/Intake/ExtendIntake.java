// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class ExtendIntake extends CommandBase {
  private Intake m_Intake = Intake.getInstance();

  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Intake m_Intake = Intake.getInstance();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtendIntake() {
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (!(m_Intake.getPistonSetting()))
    {
      m_Intake.Extend();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
