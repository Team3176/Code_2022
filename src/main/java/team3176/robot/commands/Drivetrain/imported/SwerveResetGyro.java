package team3176.robot.commands.Drivetrain.imported;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import javax.net.ssl.TrustManagerFactorySpi;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import team3176.robot.subsystems.drivetrain.Drivetrain;

/**
 * Makes the gyro's "zero point" its current position, for recallibration.
 */
public class SwerveResetGyro extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private boolean isreset;

  public SwerveResetGyro() {
    addRequirements(drivetrain);
    isreset = false;
  }

  @Override
  public void initialize() {  
  }

  @Override
  public void execute() {
    m_gyro.resetGyroHWValues();
    isreset = true;
  }

  @Override
  public boolean isFinished() { 
    if (isreset) {
      return true;
    } else {
      return false;
    } 
  }

  @Override
  public void end(boolean interrupted) { 
  }

}