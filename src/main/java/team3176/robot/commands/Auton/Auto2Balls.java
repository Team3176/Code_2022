// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Indexer.IndexerStop;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeSpin;
import team3176.robot.commands.Intake.IntakeSpint;
import team3176.robot.commands.Intake.Intaking;
import team3176.robot.commands.Shooter.FeederStop;
import team3176.robot.commands.Shooter.FlywheelStop;
import team3176.robot.commands.Shooter.FlywheelVelocityToggle;
import team3176.robot.commands.Util.TimeDelay;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto2Balls extends SequentialCommandGroup {
  /** Creates a new Auto2Balls. */
  public Auto2Balls() {
    addCommands(
      new ParallelCommandGroup(
        new ShootSetVals(),
        new FlywheelVelocityToggle()
      ),
      new TimeDelay(2), //TODO: TUNE
      new ParallelCommandGroup(
        new FlywheelStop(),
        new FeederStop(),
        new IndexerStop(),
        new IntakeSpint()
      ),
      new AutonRotate(5, -20),
      new TrapezoidDrive(8, -1),
      new Intaking(),
      new TimeDelay(2),
      new TrapezoidDrive(-8, 1),
      new AutonRotate(5, 20),
      new ParallelCommandGroup(
        new ShootSetVals(),
        new FlywheelVelocityToggle()
      ),
      new TimeDelay(2),
      new ParallelCommandGroup(
        new FlywheelStop(),
        new FeederStop(),
        new IndexerStop(),
        new IntakeSpint()
      )
    );
  }
}
