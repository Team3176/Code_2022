// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootOneBall extends SequentialCommandGroup {
  public ShootOneBall() {
    addCommands(
      new ParallelCommandGroup(new AnglerVisionSet(), new FeederRun(), new FlywheelVisionSet()),
      new IndexerPositionChange(010),
      new IndexerPositionChange(001),
      new IndexerPositionChange(000),
      new TimeDelay(2), //TODO: TUNE DOWN THE TIME
      new ParallelCommandGroup(new FeederStop(), new FlywheelStop())
    );
  }
}
