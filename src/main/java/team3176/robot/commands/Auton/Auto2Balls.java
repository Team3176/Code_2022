// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeSpintAuton;
import team3176.robot.commands.Intake.Intaking;
import team3176.robot.commands.Intake.RetractIntake;

public class Auto2Balls extends SequentialCommandGroup {
  public Auto2Balls() {
    addCommands(
      new TrapezoidDrive(7, 0),
      // new Intaking(),
      new IntakeExtendSpin(),
      new WaitCommand(1),            //TODO: TUNE
      new TrapezoidDrive(1, 0),
      new ParallelCommandGroup(
        new IntakeSpintAuton(),
        new RetractIntake()
      ),
      new AutonFenderShootSequence(),
      new WaitCommand(2),            //TODO: TUNE
      new AutonStopShootParallel()
    );
  }
}
