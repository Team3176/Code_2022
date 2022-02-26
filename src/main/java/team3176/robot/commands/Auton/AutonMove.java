// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;

public class AutonMove extends SequentialCommandGroup {
  /**
   * Robot Centric
   * @param magnitude The distance in inches
   * @param direction The direction as a int (0 - Front; pi - Back; pi/2 - Left; 3pi/2 - Right)
   */
  public AutonMove(double magnitude, double direction) {
    addCommands(
      new TrapezoidDrive(magnitude * Math.cos(direction), magnitude * Math.sin(direction))
    ); 
  }
}
