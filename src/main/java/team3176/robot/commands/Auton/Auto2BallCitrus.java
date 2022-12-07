// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.drivetrain.AutonRotate;
import team3176.robot.commands.drivetrain.SwerveResetGyro;
import team3176.robot.commands.drivetrain.TrapezoidDrive;
import team3176.robot.commands.drivetrain.*;

public class Auto2BallCitrus extends SequentialCommandGroup {
  public Auto2BallCitrus() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(6, 0),   
      new AutonRotate(.15, 4.5),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel(),

      /* Citrus Zone */
        //Line
      new ParallelCommandGroup(
        new AutonRotate(-.15, 80), //Was 90
        new AutoBallCitrusOneFlywheelAngle(),
        new IntakeExtendSpin()
      ),
      new TrapezoidDrive(5, 0),
      new ParallelCommandGroup(
        new IntakeRetractStop(),
        new AutonRotate(-.15, 48) //Was 45, 50
      ),
      
      new AutoShoot50(),
      new WaitCommand(1),
      new AutonStopShootParallel()//,
        //Mid
      // new ParallelCommandGroup(
      //   new AutoBallOneFlywheelAngle(),
      //   new AutonRotate(.15, 135),
      //   new IntakeExtendSpin()
      // ),
      // new TrapezoidDrive(14, 0),
      // new ParallelCommandGroup(
      //   new IntakeRetractStop(),
      //   new AutonRotate(-.15, 90)
      // ),
      // new AutoShoot50(),
      // new WaitCommand(0.5),
      // new AutonStopShootParallel()
    );
  }
}
