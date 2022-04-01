// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Indexer;
import team3176.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopIntakeAndIndexer extends InstantCommand {
  
  private Intake m_Intake = Intake.getInstance();
  private Indexer m_Indexer = Indexer.getInstance();
  
  public StopIntakeAndIndexer() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Indexer, m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_Intake.isExtended()) { m_Intake.Retract(); }
    m_Intake.stopMotor();
    m_Indexer.motorStop();
  }
}
