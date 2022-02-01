package team3176.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;

public class SwerveDrive extends CommandBase {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();

  private DoubleSupplier forwardCommand;
  private DoubleSupplier strafeCommand;
  private DoubleSupplier spinCommand;

  private BooleanSupplier isFieldCentric;
  private BooleanSupplier isRobotCentric;

  public SwerveDrive( DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand) {
 //                     BooleanSupplier isFieldCentric, BooleanSupplier isRobotCentric ) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    //this.isFieldCentric = isFieldCentric;
    //this.isRobotCentric = isRobotCentric;
    addRequirements(drivetrain);
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    drivetrain.setDriveMode(driveMode.DRIVE);
    drivetrain.setSpinLock(false);
  }

  @Override
  public void execute() {

    if(m_CoordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      //drivetrain.setCoordType(coordType.FIELD_CENTRIC);
      m_CoordSys.setFieldCentricOffset();
    }
    //if(isRobotCentric.getAsBoolean()) {
    //  drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    //}
    drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, strafeCommand.getAsDouble() * DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND, spinCommand.getAsDouble()*10);
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {  }
}