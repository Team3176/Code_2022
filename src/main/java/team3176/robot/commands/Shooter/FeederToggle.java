// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.FeederConstants;
import team3176.robot.subsystems.Feeder;

public class FeederToggle extends InstantCommand {
  private Feeder m_Feeder = Feeder.getInstance();
  private boolean state;
  public FeederToggle() {
    addRequirements(m_Feeder);
  }

  @Override
  public void initialize() {
    state = m_Feeder.isFeederRunning();
    if(state) {m_Feeder.stopMotor();}
    else {m_Feeder.setVelocityPID(FeederConstants.MAX_VELOCITY);}
  }
}
