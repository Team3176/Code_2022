// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.angler.Angler;

public class AnglerVisionSet extends CommandBase {
  private Angler m_Angler = Angler.getInstance();
  
  public AnglerVisionSet() {
    addRequirements(m_Angler);
  }

  @Override
  public void initialize() {
    double visionAngle = 55.0; //m_Vision.getBestAngle(); //TODO: ADD VISION
    m_Angler.moveToAngle(visionAngle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
