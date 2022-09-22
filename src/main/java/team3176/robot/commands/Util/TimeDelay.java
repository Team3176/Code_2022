// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

/**
 * TimeDelay Command to use in Command Groups to make sure that the commands have equal spacing
 */

public class TimeDelay extends CommandBase {
  private double setTime;
  private double startTime;
  private Timer clock;
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();

  public TimeDelay(double delay) {
    startTime = Timer.getFPGATimestamp();
    setTime = delay;
    clock = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    clock.start();
    m_SwerveSubsystem.drive(0, 0, 0);
  }

  @Override
  public void execute() {
    double smallnum=Math.pow(10,-20);
    m_SwerveSubsystem.drive(smallnum, smallnum, smallnum);
  }

  @Override
  public void end(boolean interrupted) {
    clock.stop();
  }

  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() > setTime + startTime) return true;
    if(clock.get() > setTime) return true;
    return false;
  }
}