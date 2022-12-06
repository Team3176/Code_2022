package team3176.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.commands.Drivetrain.imported.TrapezoidDrive;

public class Custer extends SequentialCommandGroup {
  public Custer() {
    addCommands(
      new TrapezoidDrive(10, 0)
    );
  }
}
