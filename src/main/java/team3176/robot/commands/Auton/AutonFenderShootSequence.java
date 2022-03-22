// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Util.*;

public class AutonFenderShootSequence extends SequentialCommandGroup {
  public AutonFenderShootSequence() {
    addCommands(
      new FlywheelVelocityPIDAuton(),
      new TimeDelay(2),
      new AutonShootSetVals()
    );
  }
}
