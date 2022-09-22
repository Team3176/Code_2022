// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TimeWait extends InstantCommand {
  private double time;
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();

  public TimeWait(double timeToWait) {
    this.time = timeToWait;

    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("##########################################################################################   TimeWait.initialize()");
    Timer.delay(this.time);
  }
}
