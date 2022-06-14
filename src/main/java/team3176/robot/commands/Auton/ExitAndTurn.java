package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;
import team3176.robot.commands.Drivetrain.imported.TrapezoidRotate;

public class ExitAndTurn extends SequentialCommandGroup {
  public ExitAndTurn() {
    addCommands(
      new TrapezoidDrive(10, 0),
      new TrapezoidRotate(1, 70),
      new TrapezoidDrive(10, 0));
  }
}

