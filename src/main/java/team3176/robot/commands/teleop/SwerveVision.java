
package team3176.robot.commands.teleop;

import java.util.function.DoubleSupplier;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.vision.Vision;
// import team3176.robot.util.God.PID3176;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

/* 
My hope for vision is that we'll be able to translate however we want but control of spin
will be handled by code that will make our robot constantly point at the target. If all we works
well, we could drive across the field while staying locked onto the target with automated spin in code
*/

public class SwerveVision extends CommandBase {
  private Vision m_Vision = Vision.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;

  // private PID3176 spinPID;
  private double spinOutput;

  public SwerveVision(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    addRequirements(drivetrain);

    // spinPID = new PID3176(0.3, 0.0, 0.0, 0.0);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.VISION);
  }

  @Override
  public void execute() {
    //spinOutput = spinPID.returnOutput(m_gyro.getGyroAngle(), m_gyro.getGyroAngle() + m_Vision.getBallDegrees());
    //drivetrain.drive(forwardCommand.getAsDouble(), strafeCommand.getAsDouble(), spinOutput);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}