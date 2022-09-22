
package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj2.command.CommandBase;

import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;

public class SwerveDefense extends CommandBase {
  private SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();

  public SwerveDefense() {
    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {
    SwerveSubsystem.setDriveMode(driveMode.DEFENSE);
  }

  @Override
  public void execute() {
    SwerveSubsystem.drive(0.0, 0.0, 0.0);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) { 
    SwerveSubsystem.setDriveMode(driveMode.DRIVE);
   }
}