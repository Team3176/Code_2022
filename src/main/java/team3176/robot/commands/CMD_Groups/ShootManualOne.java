// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Indexer.IndexerPositionChange;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Util.TimeDelay;

public class ShootManualOne extends SequentialCommandGroup {
  public ShootManualOne(double angle) {
    addCommands(
      new ParallelCommandGroup(new AnglerInput(angle), new FeederRun(), new FlywheelMaxVel()),
      new IndexerPositionChange(010),
      new IndexerPositionChange(001),
      new IndexerPositionChange(000),
      new TimeDelay(2), //TODO: TUNE DOWN THE TIME
      new ParallelCommandGroup(new FeederStop(), new FlywheelStop())
    );
  }
}
