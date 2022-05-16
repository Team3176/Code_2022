// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.Angler;

public class IncrementAnglerAngle extends CommandBase {
  private Angler m_Angler = Angler.getInstance();
  double startingAngle;
  double wantedAngle;

  public IncrementAnglerAngle() {
    addRequirements(m_Angler);
    this.startingAngle = m_Angler.desiredAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Angler.incrAnglerDesiredAngle();
    this.wantedAngle = m_Angler.desiredAngle;
    m_Angler.moveToAngle(this.wantedAngle);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (this.wantedAngle > this.startingAngle) {
      return true;
    } else {
    return false;
    }
  }
}