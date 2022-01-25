// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.common.TimeDelay;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToMid extends SequentialCommandGroup {
  /** Creates a new ClimbToMid. */
  public ClimbToMid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimbWinchUp(),
      new TimeDelay(5),
      new ClimbPrimaryPistonEngage(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage(),
      new TimeDelay(5),
      new ParallelCommandGroup(new ClimbWinchUp(), new ClimbPrimaryPistonRetract()),
      new TimeDelay(5),
      new ClimbPrimaryPistonEngage(),
      new TimeDelay(5),
      new ClimbSecondaryPistonRetract(),
      new TimeDelay(5),
      new ClimbWinchDown(),
      new TimeDelay(5),
      new ClimbSecondaryPistonEngage()
    );
  }
}
