// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.AutoBallOneFlywheelAngle;
import team3176.robot.commands.CMD_Groups.AutoShoot50;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeRetractStop;
import team3176.robot.commands.Shooter.AnglerZeroAtMax;
import team3176.robot.commands.drivetrain.imported.AutonRotate;
import team3176.robot.commands.drivetrain.imported.SwerveResetGyro;
import team3176.robot.commands.drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.drivetrain.imported.TrapezoidRotate;

public class Auto2Balls extends SequentialCommandGroup {
  public Auto2Balls() {
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
      new AutonStopShootParallel()
    );
  }
}
