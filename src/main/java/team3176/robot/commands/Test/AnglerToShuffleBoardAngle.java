// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.angler.Angler;

public class AnglerToShuffleBoardAngle extends CommandBase {
  /** Creates a new AnglerToShuffleBoardAngle. */
  private Angler m_angler = Angler.getInstance();

  public AnglerToShuffleBoardAngle() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_angler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("AnglerDegreesSet", 90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double exactAngle = SmartDashboard.getNumber("AnglerDegreesSet", 90);
    m_angler.moveToAngle(exactAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
