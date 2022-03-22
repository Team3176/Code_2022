// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.AnglerConstants;
import team3176.robot.subsystems.Angler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnglerPctOutput extends CommandBase {
  private Angler m_angler = Angler.getInstance();
  public AnglerPctOutput() {
    addRequirements(m_angler);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(AnglerConstants.kShuffleboardPercentName, 0.0);
  }

  @Override
  public void execute() {
    m_angler.testPercentOutput();
  }

  @Override
  public void end(boolean interrupted) {
    m_angler.engageRawMotor(0.0);
    SmartDashboard.delete(AnglerConstants.kShuffleboardPercentName);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
