
package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj2.command.CommandBase;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

public class SwerveDefenseOff extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public SwerveDefenseOff() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() { return true; }

  @Override
  public void end(boolean interrupted) {
   }
}