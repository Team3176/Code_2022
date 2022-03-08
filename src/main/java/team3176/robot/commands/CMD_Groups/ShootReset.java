// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team3176.robot.commands.Shooter.FeederStop;
import team3176.robot.commands.Shooter.FlywheelStop;

public class ShootReset extends ParallelCommandGroup {
  public ShootReset() {
    addCommands(
      new FlywheelStop(),
      new FeederStop()
    );
  }
}
