// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Indexer.*;
import team3176.robot.commands.Util.TimeDelay;
import team3176.robot.constants.ShooterLocationValues;

public class ShootBasedOnLocationValues extends SequentialCommandGroup {
  public ShootBasedOnLocationValues(int position) {
    addCommands(
      new ParallelCommandGroup(
        new AnglerInput(ShooterLocationValues.POINTS[position][0]),
        new FeederRun(),
        new FlywheelBackSpinSet(
          ShooterLocationValues.POINTS[position][1],
          ShooterLocationValues.POINTS[position][2]
        )
      ),
      new IndexerShootingMode(),
      new TimeDelay(2), //TODO: TUNE DOWN THE TIME
      new ParallelCommandGroup(
        new FeederStop(),
        new FlywheelStop()
      )
    );
  }
}