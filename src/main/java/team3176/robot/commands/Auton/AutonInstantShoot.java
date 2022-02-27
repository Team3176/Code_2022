// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Util.TimeDelay;

public class AutonInstantShoot extends SequentialCommandGroup {
  public AutonInstantShoot() {
    addCommands(
      new ParallelCommandGroup(new AnglerVisionSet(), new FeederRun(), new FlywheelVisionSet()),
      new TimeDelay(3),
      new ParallelCommandGroup(new FeederStop(), new FlywheelStop())
    );
  }
}
