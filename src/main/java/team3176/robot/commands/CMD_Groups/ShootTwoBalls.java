// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Indexer.IndexerPositionChange;
import team3176.robot.commands.Indexer.IndexerShootingMode;
import team3176.robot.commands.Shooter.AnglerVisionSet;
import team3176.robot.commands.Shooter.FeederRun;
import team3176.robot.commands.Shooter.FeederStop;
import team3176.robot.commands.Shooter.FlywheelStop;
import team3176.robot.commands.Shooter.FlywheelVisionSet;
import team3176.robot.commands.Util.TimeDelay;

public class ShootTwoBalls extends SequentialCommandGroup {
  public ShootTwoBalls() {
    addCommands(
      new ParallelCommandGroup(new AnglerVisionSet(), new FeederRun(), new FlywheelVisionSet()),
      new IndexerShootingMode(),
      new TimeDelay(4), //TODO: TUNE DOWN THE TIME
      new ParallelCommandGroup(new FeederStop(), new FlywheelStop())
    );
  }
}