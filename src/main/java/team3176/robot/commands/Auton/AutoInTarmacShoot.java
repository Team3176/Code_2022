// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoInTarmacShoot extends SequentialCommandGroup {
  public AutoInTarmacShoot() {
    addCommands(
      new AutonFenderShootSequence(),
      new WaitCommand(2), //TODO: TUNE
      new AutonStopShootParallel(),
      new AutonExitTarmac()
    );
  }
}