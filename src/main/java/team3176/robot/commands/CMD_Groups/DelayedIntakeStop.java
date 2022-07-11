// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Intake.StopIntakeAndIndexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DelayedIntakeStop extends SequentialCommandGroup {
  /** Creates a new IntakingDirectTimeFix. */
  public DelayedIntakeStop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitCommand(1),
        new StopIntakeAndIndexer()
    );
  }
}
