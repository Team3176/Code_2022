// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOneBall extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  public ShootOneBall() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexerPositionChange(010),
      // Angles
      new IndexerPositionChange(001),
      // Start Feeder and Flywheels
      new IndexerPositionChange(000)
      // Stop Stuff
    );
  }
}
