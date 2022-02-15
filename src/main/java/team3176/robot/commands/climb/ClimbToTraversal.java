// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.common.TimeDelay;

/**
 * Sequential Command Group to Climb to the Traversal Rung (if mechanically possible)
 */

public class ClimbToTraversal extends SequentialCommandGroup {
  public ClimbToTraversal() {
    addCommands(
      new ClimbWinchUp(),
      new TimeDelay(5),
      new ClimbPassivePistonEngage(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage(),
      new TimeDelay(5),
      new ParallelCommandGroup(new ClimbWinchUp(), new ClimbPassivePistonRetract()),
      new TimeDelay(5),
      new ClimbPassivePistonEngage(),
      new TimeDelay(5),
      new ClimbSecondaryPistonRetract(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage(),
      new TimeDelay(5),
      new ParallelCommandGroup(new ClimbWinchUp(), new ClimbPassivePistonRetract()),
      new TimeDelay(5),
      new ClimbPassivePistonEngage(),
      new TimeDelay(5),
      new ClimbSecondaryPistonRetract(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage()
    );
  }
}
