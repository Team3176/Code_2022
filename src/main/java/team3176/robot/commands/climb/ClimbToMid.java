// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.common.TimeDelay;

/**
 * Sequential Command Group to Climb to the Mid Rung 
 */

public class ClimbToMid extends SequentialCommandGroup {
  public ClimbToMid() {
    addCommands(
      new ClimbWinchUp(),
      new TimeDelay(5),
      new ClimbPassivePistonEngage(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage()
    );
  }
}