// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbToTraversal extends CommandBase {
  /** Creates a new ClimbToTraversal. */
  public ClimbToTraversal() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
    /**
     * m_Climb.winchUp();
     * m_Climb.primaryPistonsEngage();
     * m_Climb.winchDown();
     * m_Climb.secondaryPistonsEngage();
     * winchUp              //
     * primaryPistonRetract //
     * primaryPistonEngage
     * secondaryPistonRetract
     * WinchDown
     * secondaryPistonEngage
     * winchUp              //
     * primaryPistonRetract //
     * primaryPistonEngage
     * secondaryPistonRetract
     * winchDown
     * secondaryPistonEngage
     */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
