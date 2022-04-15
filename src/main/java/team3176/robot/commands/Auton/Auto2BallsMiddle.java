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
import team3176.robot.commands.Vision.VisionSpinCorrectionOff;
import team3176.robot.commands.Vision.VisionSpinCorrectionOn;

public class Auto2BallsMiddle extends SequentialCommandGroup {
  public Auto2BallsMiddle() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallTwoFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(6, 0),
      new FlywheelAngleVisionAuton(),
      new TrapezoidRotate(-1,7), //Was 5
      //new AutonRotate(.15, 7),
      new WaitCommand(0.5),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopBeltsParallel() 

    );
  }
}
