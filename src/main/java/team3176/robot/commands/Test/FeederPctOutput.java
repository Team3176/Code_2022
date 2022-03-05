// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.FeederConstants;
import team3176.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederPctOutput extends CommandBase {
  private Feeder m_Transfer = Feeder.getInstance();
  public FeederPctOutput() {
    addRequirements(m_Transfer);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(FeederConstants.kShuffleboardPercentName, 0.0);
  }

  @Override
  public void execute() {
    m_Transfer.percentOutput();
  }

  @Override
  public void end(boolean interrupted) {
    m_Transfer.stopMotor();
    SmartDashboard.delete(FeederConstants.kShuffleboardPercentName);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
