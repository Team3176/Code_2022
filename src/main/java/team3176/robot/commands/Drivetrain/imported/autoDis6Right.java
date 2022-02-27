// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class autoDis6Right extends SequentialCommandGroup {
  /** Creates a new autoDis6Right. */
  public autoDis6Right() {
    addCommands(
      new TrapezoidDrive(-1, 6)
    );
  }
}
