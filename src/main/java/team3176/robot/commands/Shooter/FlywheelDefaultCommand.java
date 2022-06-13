// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.flywheel.Flywheel;

public class FlywheelDefaultCommand extends CommandBase {
  private Flywheel m_Flywheel = Flywheel.getInstance();
  private double pctOne, pctTwo;
  public FlywheelDefaultCommand(double pctOne, double pctTwo) {
    addRequirements(m_Flywheel);
    this.pctOne = pctOne;
    this.pctTwo = pctTwo;

  }

  @Override
  public void initialize() {
    if(m_Flywheel.getAutoSpinFlywheels()) {
      m_Flywheel.spinMotorsVelocityPID(pctOne, pctTwo);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
