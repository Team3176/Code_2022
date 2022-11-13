// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain.imported;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.drivetrain.imported.vision_control.AlignVizDistBangBang;
import team3176.robot.subsystems.vision.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonBatteryOne extends SequentialCommandGroup { //TODO: WIP for AimLock
  //Vision m_Vision = Vision.getInstance();
  
  public AutonBatteryOne() {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new RotateUntilTargetRecogd(),
      new AlignVizDistBangBang(15)
    );
  }
  
}