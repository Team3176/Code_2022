package team3176.robot.commands.Drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys.coordType;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;
import team3176.robot.subsystems.SwerveSubsystem.Gyro3176;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys;
public class SwerveOrbit extends CommandBase {
  private SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();
  private DoubleSupplier orbitSpeed;
  private DoubleSupplier pov;

  private double orbitEtherRadius = 30.0; // inches

  private boolean wasFieldCentric;

  private double radianOffset;

  public SwerveOrbit(DoubleSupplier orbitSpeed, DoubleSupplier pov) {
    this.orbitSpeed = orbitSpeed;
    this.pov = pov;
    addRequirements(SwerveSubsystem);
  }

  @Override
  public void initialize() {
    if(m_coordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC) {
      wasFieldCentric = true;
    } else {
      wasFieldCentric = false;
    }
    radianOffset = m_gyro.getCurrentChassisYaw() - m_coordSys.getFieldCentricOffset();

    // SmartDashboard.putNumber("currentAngle", SwerveSubsystem.getCurrentAngle());
    // SmartDashboard.putNumber("getFieldCentricOffset", SwerveSubsystem.getFieldCentricOffset());
    // SmartDashboard.putNumber("radianOffset", radianOffset);

    SwerveSubsystem.setDriveMode(driveMode.ORBIT);
    m_coordSys.setCoordType(coordType.ROBOT_CENTRIC);
  }

  @Override
  public void execute() {

    double forwardCommand = orbitSpeed.getAsDouble() * Math.cos(radianOffset);
    double strafeCommand = orbitSpeed.getAsDouble() * Math.sin(radianOffset);

    if(pov.getAsDouble() == 45.0 || pov.getAsDouble() == 90.0 || pov.getAsDouble() == 135.0) { // If on right side
      // Orbit Clockwise
      SwerveSubsystem.drive(forwardCommand, strafeCommand, orbitSpeed.getAsDouble() / orbitEtherRadius /* inches */);
    } 
    else if (pov.getAsDouble() == 225.0 || pov.getAsDouble() == 270.0 || pov.getAsDouble() == 315.0) { // If on left side
      // Orbit Counter-Clockwise
      SwerveSubsystem.drive(forwardCommand, strafeCommand, -orbitSpeed.getAsDouble() / orbitEtherRadius /* inches */);
    }
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    if(wasFieldCentric) {

      m_coordSys.setCoordType(coordType.FIELD_CENTRIC);
    } else {
      m_coordSys.setCoordType(coordType.ROBOT_CENTRIC);
    }
  }
}
