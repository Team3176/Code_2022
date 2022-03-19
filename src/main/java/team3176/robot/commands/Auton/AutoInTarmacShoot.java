// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Indexer.IndexerStop;
import team3176.robot.commands.Intake.IntakeSpint;
import team3176.robot.commands.Shooter.FeederStop;
import team3176.robot.commands.Shooter.FlywheelBackspinVelocityPID;
import team3176.robot.commands.Shooter.FlywheelStop;
import team3176.robot.commands.Shooter.FlywheelVelocityPID;
import team3176.robot.commands.Shooter.FlywheelVelocityToggle;
import team3176.robot.commands.Util.TimeDelay;

public class AutoInTarmacShoot extends SequentialCommandGroup {
  public AutoInTarmacShoot() {
    addCommands(
      new SequentialCommandGroup(
        new FlywheelVelocityPID(),
        new TimeDelay(2),
        new AutonShootSetVals()
      ),
      new TimeDelay(2), //TODO: TUNE
      new ParallelCommandGroup(
        new FlywheelStop(),
        new FeederStop(),
        new IndexerStop(),
        new IntakeSpint()
      ),
      new TrapezoidDrive(8, -1)
    );
  }
}