// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.AnglerConstants;
import team3176.robot.subsystems.shooter.Angler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnglerPctOutput extends CommandBase {
  /** Creates a new AnglerPctOutput. */

  private Angler m_angler = Angler.getInstance();

  public AnglerPctOutput() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_angler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(AnglerConstants.kShuffleboardPercentName, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_angler.testPercentOutput();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.engageRawMotor(0.0);
    SmartDashboard.delete(AnglerConstants.kShuffleboardPercentName);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
