// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.*;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.drivetrain.imported.*;

public class AutoInterfere extends SequentialCommandGroup {
  public AutoInterfere() {
    addCommands(
      //new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new TrapezoidRotate(-1,22.5),
      //new AutonRotate(.15, 100),
      new IntakeExtendReverse(),
      new WaitCommand(0.5),
      new IntakeRetractStop(),
      new AutonStopShootParallel(),

      //Score
      new AutoBallOneFlywheelAngle(),
      new TrapezoidRotate(-1,22.5),
      //new AutonRotate(.15, 150),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),
      new IntakeRetractStop(),
      new TrapezoidRotate(-1,20),
      //new AutonRotate(.15, 90),
      new AutoShoot50(),
      new WaitCommand(1.2),
      new AutonStopShootParallel()

    );
  }
}