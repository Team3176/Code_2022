// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Intake.Intaking;

public class Auto2Balls extends SequentialCommandGroup {
  public Auto2Balls() {
    addCommands(
      new AutonShootSequence(),
      new WaitCommand(2),            //TODO: TUNE
      new AutonStopShootParallel(),
      new AutonRotate(5, -20),        //
                                          //  Combine into a Move CMD
      new TrapezoidDrive(8, -1),      //
      new Intaking(),
      new WaitCommand(2),            //TODO: TUNE
      new TrapezoidDrive(-8, 1),      //
                                          //  Combine into a Move CMD
      new AutonRotate(5, 20),         //
      new AutonShootSequence(),
      new WaitCommand(2),            //TODO: TUNE
      new AutonStopShootParallel()
    );
  }
}
