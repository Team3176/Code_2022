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

public class SwervePivotAtPod extends CommandBase {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private DoubleSupplier forwardCommand, strafeCommand, spinCommand;
  private Double hatPov, pov;

  private double radianOffset;

  public SwervePivotAtPod(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand, Double hatPov) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    this.hatPov = hatPov;
    addRequirements(m_Drivetrain, m_Gyro);
  }

  @Override
  public void initialize() {
    double yaw = m_Gyro.getGyroAngle_inDegrees();
    if ((yaw >= 45 && yaw < 135) || (yaw >= 315)) {
      switch (this.hatPov.intValue()) {
        case 45:  this.pov = this.pov + 90.0;
                  break;
        case 135: this.pov = this.pov + 90.0;
                  break;
        case 225: this.pov = this.pov + 90.0;
                  break;
        case 315: this.pov = 45.0;
                  break;
      }
    }
    
    if (yaw >= 135 && yaw < 225) {
      switch (this.hatPov.intValue()) {
        case 45:  this.pov = this.pov + 180.0;
                  break;
        case 135: this.pov = this.pov + 180.0;
                  break;
        case 225: this.pov = 45.0;
                  break;
        case 315: this.pov = 135.0;
                  break;
      }
    } 

    if (yaw >= 225 && yaw < 315) {
      switch (this.hatPov.intValue()) {
        case 45:  this.pov = this.pov + 315.0;
                  break;
        case 135: this.pov = 45.0;
                  break;
        case 225: this.pov = 135.0;
                  break;
        case 315: this.pov = 225.0;
                  break;
      }
    }


    radianOffset = m_Gyro.getCurrentChassisYaw() - m_CoordSys.getFieldCentricOffset();

  }

  @Override
  public void execute() {


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

    m_Drivetrain.drive(forwardCommand.getAsDouble() * DrivetrainConstants.MAX_ACCEL_FEET_PER_SECOND, strafeCommand.getAsDouble() * DrivetrainConstants.MAX_ACCEL_FEET_PER_SECOND, spinCommand.getAsDouble() * 25 /* inches */); //}
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    //if(wasFieldCentric) {

    m_Drivetrain.setDriveMode(driveMode.DRIVE);
  }
}