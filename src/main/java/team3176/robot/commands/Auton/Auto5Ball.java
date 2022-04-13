// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.*;
import team3176.robot.commands.Indexer.IndexerStop;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Vision.VisionSpinCorrectionOff;
import team3176.robot.commands.Vision.VisionSpinCorrectionOn;

public class Auto5Ball extends SequentialCommandGroup {
  public Auto5Ball() {
    addCommands(
      new SwerveResetGyro(),
      // new AnglerZeroAtMax(),
      new AnglerSetMaxZero(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new TrapezoidRotate(1,5),
      //new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(1.1),
      new AutonStopBeltsParallel(),

          /* ROTATE AND MOVE TO 2nd BALL */
      
      new AutoBallThreeFlywheelAngle(),
      new TrapezoidRotate(1,10),
      //new AutonRotate(.15, 87),  //80
      new IntakeExtendSpin(),
      new TrapezoidDrive(11, 0),
      // new WaitCommand(1),
          /* AUTON 3BALL ZONE */
      new IntakeRetractStop(),
      new TrapezoidRotate(-1,7),
      //new AutonRotate(-.15, 58),
      // new WaitCommand(2),
      new AutoShoot50(),
      new WaitCommand(1.1),
      new AutonStopBeltsParallel(),

      /** 4 Ball + 5 Ball Delay */

      // new AutoBallFourFlywheelAngle(),
      // new IntakeExtendSpin(),
      new IntakingAuton(),
      // new TrapezoidDrive(0, 4), //TODO: FIND DISTANCE
      new TrapezoidRotate(1, 8),  //TODO: FIND ANGLE
      //new AutonRotate(.15, 28),  //TODO: FIND ANGLE
      new TrapezoidDrive(18, 0), //TODO: FIND DISTANCE
      new WaitCommand(1), //5 Ball Difference for loading in the gap between the Intake and Indexer.
      new TrapezoidRotate(-1,9), //TODO: FIND ANGLE (a pure 50 deg whould prob overshoot)
      //new AutonRotate(-.15, 40), //TODO: FIND ANGLE (a pure 50 deg whould prob overshoot)
      // new TrapezoidDrive(.4, 0),
      new WaitCommand(1),
      new IndexerStop(),
      new IntakeRetractStop(),
      new TrapezoidRotate(1, 10),
      //new AutonRotate(.15, 40),
      new TrapezoidDrive(-9, 0),
      new TrapezoidRotate(-1,8),
      //new AutonRotate(-.15, 30),
      new ParallelCommandGroup(
        new FlywheelAngleVisionAuton(),
        new VisionSpinCorrectionOn()
      ),
      new AlignVizYawSpinCorrection(),
      new AutoShoot50(),
      new WaitCommand(2),
      new ParallelCommandGroup(
        new VisionSpinCorrectionOff(),
        new AutonStopShootParallel()
      )
    );
  }
}