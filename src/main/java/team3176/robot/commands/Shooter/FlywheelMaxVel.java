// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.FlywheelConstants;
import team3176.robot.subsystems.flywheel.Flywheel;

public class FlywheelMaxVel extends InstantCommand {
  private Flywheel m_Flywheel = Flywheel.getInstance();

  public FlywheelMaxVel() {
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    m_Flywheel.spinMotors(FlywheelConstants.MAX_TICKSPER100MS);
  }
}
