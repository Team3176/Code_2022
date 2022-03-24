// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.AutoBallOneFlywheelAngle;
import team3176.robot.commands.CMD_Groups.AutoBallTwoFlywheelAngle;
import team3176.robot.commands.CMD_Groups.AutoShoot50;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeRetractStop;
import team3176.robot.commands.Shooter.AnglerZeroAtMax;
import team3176.robot.commands.Vision.VisionSpinCorrectionOff;
import team3176.robot.commands.Vision.VisionSpinCorrectionOn;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;

public class Auto3Balls extends SequentialCommandGroup {
  public Auto3Balls() {
    addCommands(
      new AnglerZeroAtMax(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(3),
      new AutonStopShootParallel(),

          /* AUTON 3BALL ZONE */

      new AutoBallTwoFlywheelAngle(),
      new IntakeExtendSpin(),
      new TrapezoidDrive(-7, 13),
      new WaitCommand(1), 
      //new VisionSpinCorrectionOn(),
      // new AutonRotate(.15, 20),
      new IntakeRetractStop(),
      // new VisionSpinCorrectionOn(),
      new AutoShoot50(),
      new WaitCommand(3),
      new AutonStopShootParallel()
      // new VisionSpinCorrectionOff()


    );
  }
}
