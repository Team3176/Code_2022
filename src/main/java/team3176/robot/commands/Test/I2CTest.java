// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class I2CTest extends InstantCommand {
  Indexer m_Indexer = Indexer.getInstance();
  public I2CTest() {
    addRequirements(m_Indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Indexer.I2CReciever();
  }
}
