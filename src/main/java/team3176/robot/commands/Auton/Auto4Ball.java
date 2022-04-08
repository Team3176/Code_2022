// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.*;
import team3176.robot.commands.Indexer.IndexerForward;
import team3176.robot.commands.Indexer.IndexerStop;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;

public class Auto4Ball extends SequentialCommandGroup {
  public Auto4Ball() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallTwoFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(6, 0),
      new TrapezoidRotate(-1,5),
      //new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel(),
      new TrapezoidRotate(1, 4),
 
      /** 4 Ball */

      new AutoBallTwoFlywheelAngle(),
      new IntakeExtendSpin(),
      new IndexerForward(),
      new TrapezoidDrive(19, 0), //TODO: FIND DISTANCE
      new WaitCommand(1.5),
      new IntakeRetractStop(),
      new IndexerStop(),
      new TrapezoidDrive(-17, 0),
      new TrapezoidRotate(-1, 6),
      //new AutonRotate(.15, 50), //TODO: FIND ANGLE (a pure 50 deg whould prob overshoot)
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel()
    );
  }
}
