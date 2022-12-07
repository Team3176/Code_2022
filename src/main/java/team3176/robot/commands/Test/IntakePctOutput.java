// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.IntakeConstants;
import team3176.robot.subsystems.intake.Intake;

public class IntakePctOutput extends CommandBase {
  /** Creates a new IntakePctOutput. */
  private Intake m_intake = Intake.getInstance();

  public IntakePctOutput() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(IntakeConstants.kShuffleboardPercentName, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.shuffleboardPercentOutput();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor();
    /* Commented b/c "delete" is depreciated from SmartDashboard.  TODO: Convert these to NT4 pub/sub model 
    SmartDashboard.delete(IntakeConstants.kShuffleboardPercentName);
    */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
