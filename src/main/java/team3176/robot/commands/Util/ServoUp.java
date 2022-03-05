// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.ServoSubsystem;

public class ServoUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ServoSubsystem servo_subsystem;

  public ServoUp(ServoSubsystem subsystem) {
    servo_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    servo_subsystem.open();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
