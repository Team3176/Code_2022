package team3176.robot.commands.Drivetrain.imported;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;

public class SwervePodsAzimuthGoHome extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();

  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;

  public SwervePodsAzimuthGoHome()  {
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
  }

  @Override
  public void execute() {

    drivetrain.sendPodsAzimuthToHome();
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setCurrentPodPosAsHome();
  }
}