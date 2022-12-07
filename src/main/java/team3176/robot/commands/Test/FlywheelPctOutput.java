// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.FlywheelConstants;
import team3176.robot.subsystems.flywheel.Flywheel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelPctOutput extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  public FlywheelPctOutput() {
    addRequirements(m_Flywheel);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
    SmartDashboard.putNumber(FlywheelConstants.kShuffleboardPercentName2, 0.0);
  }

  @Override
  public void execute() {
    m_Flywheel.percentOutput_1();
    m_Flywheel.percentOutput_2();
  }

  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
    /* Commented b/c "delete" is depreciated from SmartDashboard.  TODO: Convert these to NT4 pub/sub model 
    SmartDashboard.delete(FlywheelConstants.kShuffleboardPercentName1);
    SmartDashboard.delete(FlywheelConstants.kShuffleboardPercentName2);
    */
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
