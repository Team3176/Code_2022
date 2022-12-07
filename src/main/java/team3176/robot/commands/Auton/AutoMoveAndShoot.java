// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.drivetrain.TrapezoidDrive;
import team3176.robot.commands.drivetrain.*;

public class AutoMoveAndShoot extends SequentialCommandGroup {
  public AutoMoveAndShoot() {
    addCommands(
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(10),
      new TrapezoidDrive(5, 0),
      new AutoShoot50(),
      new WaitCommand(3),
      new AutonStopShootParallel()
    );
  }
}
