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

public class Auto4Ball extends SequentialCommandGroup {
  public Auto4Ball() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new TrapezoidRotate(1,5),
      //new AutonRotate(.15, 7),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel(),

          /* ROTATE AND MOVE TO 2nd BALL */
      
      new TrapezoidRotate(1,22.5),
      //new AutonRotate(.15,120),
      new IntakeExtendSpin(),
      new TrapezoidDrive(15, 0),
      new AutoBallTwoFlywheelAngle(),
      new WaitCommand(1),
          /* AUTON 3BALL ZONE */
      new IntakeRetractStop(),
      new TrapezoidRotate(-1,10),
      //new AutonRotate(-1.5, 40),
      // new WaitCommand(2),
      new AutoShoot50(),
      new WaitCommand(1),
      new AutonStopShootParallel(),

      /** 4 Ball */
      new FlywheelAngleWall(),
      new IntakeExtendSpin(),
      new TrapezoidDrive(0, 4), //TODO: FIND DISTANCE
      // new AutonRotate(-.15, 45),  //TODO: FIND ANGLE Not Sure I We Even Need (if we do it should be small)
      new TrapezoidDrive(14, 0), //TODO: FIND DISTANCE
      new IntakeRetractStop(),
      new TrapezoidRotate(1,11),
      //new AutonRotate(.15, 50), //TODO: FIND ANGLE (a pure 50 deg whould prob overshoot)
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel()
    );
  }
}
