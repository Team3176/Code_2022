// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDelay extends CommandBase {
  double setTime;
  // double startTime;
  Timer clock;
  public TimeDelay(double delay) {
    setTime = delay;
    clock = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startTime = Timer.getFPGATimestamp();
    clock.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clock.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(clock.get() >= setTime) return true;
    return false;
  }
}
