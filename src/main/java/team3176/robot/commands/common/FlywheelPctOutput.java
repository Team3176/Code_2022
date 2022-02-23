// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.shooter.Flywheel;
import team3176.robot.constants.FlywheelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelPctOutput extends CommandBase {
  /** Creates a new FlywheelPctOutput. */

  private Flywheel m_Flywheel = Flywheel.getInstance();

  public FlywheelPctOutput() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
    SmartDashboard.putNumber(FlywheelConstants.kShuffleboardPercentName1, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Flywheel.percentOutput_1();
    m_Flywheel.percentOutput_2();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
    SmartDashboard.delete(FlywheelConstants.kShuffleboardPercentName1);
    SmartDashboard.delete(FlywheelConstants.kShuffleboardPercentName2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
