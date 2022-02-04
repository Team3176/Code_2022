// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import team3176.robot.commands.teleop.VisionToggleLeds;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunAutonMutable1 extends SequentialCommandGroup {
  /** Creates a new RunAuton. */
  public RunAutonMutable1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new VisionToggleLeds(),
      // new AlignVizDistBangBang(10)
      //new AlignVizYawPLoop(),
      //new VisionToggleLeds()
    new ShootVisionSetUp()
    );
  }
}