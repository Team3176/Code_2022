package team3176.robot.commands.Drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import team3176.robot.subsystems.drivetrain.CoordSys;

public class SwerveRotateAtPod extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();
  private DoubleSupplier forwardCommand, strafeCommand, spinCommand;
  private Double pov;

  private double orbitEtherRadius = 30.0; // inches

  private boolean wasFieldCentric;

  private double radianOffset;

  public SwerveRotateAtPod(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand, Double pov) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    this.pov = pov;
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    //if(m_coordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
    //  wasFieldCentric = true;
    //} else {
    //  wasFieldCentric = false;
   // }
    radianOffset = m_gyro.getCurrentChassisYaw() - m_coordSys.getFieldCentricOffset();

    // SmartDashboard.putNumber("currentAngle", drivetrain.getCurrentAngle());
    // SmartDashboard.putNumber("getFieldCentricOffset", drivetrain.getFieldCentricOffset());
    // SmartDashboard.putNumber("radianOffset", radianOffset);

    //m_coordSys.setCoordType(coordType.ROBOT_CENTRIC);
  }

  @Override
  public void execute() {

    //double forwardCommand = orbitSpeed.getAsDouble() * Math.cos(radianOffset);
    //double strafeCommand = orbitSpeed.getAsDouble() * Math.sin(radianOffset);

    if (pov == 45.0) {
      m_Drivetrain.setDriveMode(driveMode.PIVOTFR);
    }

    if (pov == 135.0) {
      m_Drivetrain.setDriveMode(driveMode.PIVOTFL);
    }

    if (pov == 225.0) {
      m_Drivetrain.setDriveMode(driveMode.PIVOTBL);
    }

    if (pov == 315.0 ) {
      m_Drivetrain.setDriveMode(driveMode.PIVOTBR);
    }

      // Orbit Clockwise
      //drivetrain.drive(forwardCommand, strafeCommand, orbitSpeed.getAsDouble() / orbitEtherRadius /* inches */);
    //} 
    //else if (pov == 225.0 || pov == 270.0 || pov == 315.0) { // If on left side
      // Orbit Counter-Clockwise
    m_Drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_ACCEL_FEET_PER_SECOND, strafeCommand.getAsDouble() * DrivetrainConstants.MAX_ACCEL_FEET_PER_SECOND, spinCommand.getAsDouble() * 10 /* inches */);
    //}
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    //if(wasFieldCentric) {

     // m_coordSys.setCoordType(coordType.FIELD_CENTRIC);
    //} else {
     // m_coordSys.setCoordType(coordType.ROBOT_CENTRIC);
   //// }
    m_Drivetrain.setDriveMode(driveMode.DRIVE);
  }
}
