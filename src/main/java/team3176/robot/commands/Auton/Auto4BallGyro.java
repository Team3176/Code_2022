// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.*;
import team3176.robot.commands.Drivetrain.imported.vision_control.AlignVizYawSpinCorrection;
import team3176.robot.commands.Indexer.IndexerForward;
import team3176.robot.commands.Indexer.IndexerStop;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;

public class Auto4BallGyro extends SequentialCommandGroup {
  public Auto4BallGyro() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallTwoFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(6, 0),
      new FlywheelAngleVisionAuton(),
      // new TrapezoidRotate(-1,7), //Was 5
      new AutonRotatePidGyro(-10),
      //new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new WaitCommand(0.5),
      new AutoShoot50(), // shoot first 2 bals
      new WaitCommand(1.5),
      new AutonStopBeltsParallel(),
      // new TrapezoidRotate(1, 6.5), //TODO: FIX ROTATION! (Was 1, 7, and earlier was 4, 6)
      new AutonRotatePidGyro(19),
 
      /** 4 Ball */

      new AutoBallTwoFlywheelAngle(),
      new IntakeExtendSpin(),
      new IndexerForward(),
      new TrapezoidDrive(16.4, 0),
      new TrapezoidDrive(2.0, 0),
      new AutonRotatePidGyro(-4),
      new WaitCommand(1.5),
      new IntakeRetractStop(),
      new IndexerStop(),
      new AutonRotatePidGyro(4),
      new FlywheelAngleVisionAuton(),
      new TrapezoidDrive(-17, 0),
      // new TrapezoidRotate(-1, 6),
      new AutonRotatePidGyro(-18),
      // new VisionSpinCorrectionOn(),
      // new FlywheelAngleVisionAuton(),
      // new TrapezoidRotate(-1,2),
      // new AutonRotatePidGyro(-4),
      new AlignVizYawSpinCorrection(),
      // new AutonRotate(.15, 50), //TODO: FIND ANGLE (a pure 50 deg whould prob overshoot)
      new AutoShoot50(),
      // new VisionSpinCorrectionOff(),
      new WaitCommand(2),
      new AutonStopShootParallel()
    );
  }
}
