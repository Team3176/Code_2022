package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.drivetrain.SwerveTimedDrive;
import team3176.robot.commands.drivetrain.TrapezoidDrive;
import team3176.robot.commands.drivetrain.TrapezoidRotate;

public class ExitAndTurn extends SequentialCommandGroup {
  public ExitAndTurn() {
    addCommands(
      new TrapezoidDrive(10, 0),
      new SwerveTimedDrive(0.0, 0.0, .2).withTimeout(2),
      new TrapezoidDrive(10, 0));
  }
}

