// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3176.robot.commands.CMD_Groups.FlywheelAngleVision;
import team3176.robot.commands.CMD_Groups.FlywheelAngleVisionAuton;
import team3176.robot.commands.CMD_Groups.ShootVision;
import team3176.robot.commands.Drivetrain.imported.AutonRotate;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Intake.IntakeExtendSpin;
import team3176.robot.commands.Intake.IntakeSpintAuton;
import team3176.robot.commands.Intake.Intaking;
import team3176.robot.commands.Intake.RetractIntake;
import team3176.robot.commands.Vision.VisionSpinCorrectionOff;
import team3176.robot.commands.Vision.VisionSpinCorrectionOn;
import team3176.robot.commands.Vision.VisionZoom2x;

public class Auto2Balls extends SequentialCommandGroup {
  public Auto2Balls() {
    addCommands(
      new VisionZoom2x(),
      new IntakeExtendSpin(),
      new TrapezoidDrive(5, 0),  
      new WaitCommand(1), 
      //new VisionSpinCorrectionOn(),
      new TrapezoidDrive(-2, 0),
      // new AutonRotate(.15, 10),
      new IntakeSpintAuton(),
      new RetractIntake(),
      new VisionSpinCorrectionOn(),
      new AutonVisionShootSequence(),
      new WaitCommand(5),            //TODO: TUNE
      new AutonStopShootParallel(),
      new VisionSpinCorrectionOff()
    );
  }
}
