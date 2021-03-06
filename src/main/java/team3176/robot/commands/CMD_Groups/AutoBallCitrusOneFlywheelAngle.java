// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.angler.Angler;
import team3176.robot.subsystems.flywheel.Flywheel;

public class AutoBallCitrusOneFlywheelAngle extends InstantCommand {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private Angler m_Angler = Angler.getInstance();
  public AutoBallCitrusOneFlywheelAngle() {
    addRequirements(m_Flywheel, m_Angler);
  }

  @Override
  public void initialize() {
    m_Flywheel.spinMotorsVelocityPID(0.28, 0.18);
    m_Angler.moveToAngle(55);
  }
}