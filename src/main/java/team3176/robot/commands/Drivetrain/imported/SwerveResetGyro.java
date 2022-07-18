package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.subsystems.drivetrain.Gyro3176;

/**
 * Makes the gyro's "zero point" its current position, for recallibration.
 */
public class SwerveResetGyro extends InstantCommand {
  public SwerveResetGyro() {
    super(()->Gyro3176.getInstance().resetGyroHWValues());
  }
}

