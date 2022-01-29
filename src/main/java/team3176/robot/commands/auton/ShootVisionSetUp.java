// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.subsystems.vision.Vision;
//import team3176.robot.commands.auton.*;
/*
import team3176.robot.commands.teleop.ShootVision;
import team3176.robot.commands.teleop.TransferDown;
import team3176.robot.commands.teleop.TransferUp;
import team3176.robot.commands.teleop.ShootReset;
*/


public class ShootVisionSetUp extends SequentialCommandGroup {
  //Vision m_Vision = Vision.getInstance();
  
  public ShootVisionSetUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      //new ShootVision(),
      /*
      new AlignVizYawBangBang(),
      // new AlignVizYawPLoop(),
      //new TransferDown(),
      // new IrCounter(3),
      new DelayCommand(5),
      new TransferUp(),
      new ShootReset(),
      */
    );
  }

}
