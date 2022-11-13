// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.drivetrain.imported.TrapezoidDrive;

public class AutonExitTarmac extends SequentialCommandGroup {
  public AutonExitTarmac() {
    addCommands(
      new TrapezoidDrive(10, 0)
    );
  }
}
