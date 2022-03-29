// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.AutoBallOneFlywheelAngle;
import team3176.robot.commands.CMD_Groups.AutoShoot50;
import team3176.robot.commands.CMD_Groups.FlywheelAngleWall;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeRetractStop;
import team3176.robot.commands.Shooter.AnglerZeroAtMax;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;
import team3176.robot.commands.Drivetrain.imported.SwerveResetGyro;


public class Auton3BallAtHanger extends SequentialCommandGroup {
  public Auton3BallAtHanger() {
    addCommands(
      new SwerveResetGyro(),
      new AnglerZeroAtMax(),
      new AutoBallOneFlywheelAngle(),
      new WaitCommand(0.5),
      new IntakeExtendSpin(),
      new TrapezoidDrive(6, 0),   
      new AutonRotate(-.15, 5),
      new IntakeRetractStop(),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel(),

      /**3 Ball */

      new FlywheelAngleWall(),
      new AutonRotate(-.15, 80),
      new IntakeExtendSpin(),
      new TrapezoidDrive(26, 0),
      new IntakeRetractStop(),
      
      new AutonRotate(.15, 10),
      new WaitCommand(3),
      new AutoShoot50(),
      new WaitCommand(2),
      new AutonStopShootParallel()
    );
  }
}
