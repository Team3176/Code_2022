// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * TimeDelay Command to use in Command Groups to make sure that the commands have equal spacing
 */

public class TimeDelay extends CommandBase {
  private double setTime;
  private Timer clock;

  public TimeDelay(double delay) {
    setTime = delay;
    clock = new Timer();
  }

  @Override
  public void initialize() {
    clock.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    clock.stop();
  }

  @Override
  public boolean isFinished() {
    if(clock.get() >= setTime) return true;
    return false;
  }
}