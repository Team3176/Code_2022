// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Drivetrain.imported.*;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;

public class Auto3Balls extends SequentialCommandGroup {
  public Auto3Balls() {
    addCommands(
      new SwerveResetGyro(),
      // new AnglerZeroAtMax(),
      new AnglerSetMaxZero(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new TrapezoidRotate(1,5),
      // new SwerveDefenseOn(),
      //new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(1.1),
      new AutonStopShootParallel(),

          /* ROTATE AND MOVE TO 2nd BALL */
      
      new AutoBallThreeFlywheelAngle(),
      // new SwerveDefenseOff(),
      new TrapezoidRotate(1,17.75),
      //new AutonRotate(.15, 87),  //80
      new IntakeExtendSpin(),
      new TrapezoidDrive(12, 0),
      // new WaitCommand(1),
          /* AUTON 3BALL ZONE */
      new IntakeRetractStop(),
      new TrapezoidRotate(-1,13),
      // new SwerveDefenseOn(),
      //new AutonRotate(-.15, 58),
      // new WaitCommand(2),
      new AutoShoot50(),
      new WaitCommand(1.1),
      new AutonStopShootParallel()//,
      // new SwerveDefenseOff()
    );
  }
}
