// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Indexer.*;
import team3176.robot.commands.Intake.*;

public class AutonStopShootParallel extends ParallelCommandGroup {
  public AutonStopShootParallel() {
    addCommands(
      new FlywheelStop(),
      new FeederStop(),
      new IndexerStop(),
      new IntakeSpintAuton()
    );
  }
}
