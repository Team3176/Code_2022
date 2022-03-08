// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Intake.*;

public class Auton5Balls extends SequentialCommandGroup {
  public Auton5Balls() {
    addCommands(
      new AutonExitTarmac(),
      new IntakeIndexerIntegration(),
      new IntakeSpint(),
      new ShootTwoBalls(),
      new AutonMove(1, 1), //TODO: FIND DISTANCE AND ANGLE
      new IntakeIndexerIntegration(),
      new IntakeSpint(),
      new AutonMove(1, 1), //TODO: FIND DISTANCE AND ANGLE
      new IntakeIndexerIntegration(),
      new IntakeSpint(),
      new ShootTwoBalls(),
      new ShootOneBall()
    );
  }
}
