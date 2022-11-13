/*
/*
 
   ____  _            _           _     ____           _ 
  |  _ \(_)_   _____ | |_    __ _| |_  |  _ \ ___   __| |
  | |_) | \ \ / / _ \| __|  / _` | __| | |_) / _ \ / _` |
  |  __/| |\ V / (_) | |_  | (_| | |_  |  __/ (_) | (_| |
  |_|   |_| \_/ \___/ \__|  \__,_|\__| |_|   \___/ \__,_|
                                                         
 
*/
package team3176.robot.commands.drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import team3176.robot.subsystems.drivetrain.CoordSys;

public class SwervePivotAtPodQuad extends CommandBase {
    private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private DoubleSupplier forwardCommand, strafeCommand, spinCommand;
  private Double hatPov, pov;

  private double radianOffset;

  public SwervePivotAtPodQuad(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand, Double hatPov) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    this.hatPov = hatPov;
    this.pov = this.hatPov;
    addRequirements(m_Drivetrain, m_Gyro);
  }

  @Override
  public void initialize() {
    double mYaw = 0;
    double yaw = m_Gyro.getYaw();

    if (yaw < 0) { mYaw = yaw % -180.0; } else { mYaw = yaw % 180;}

    if ((mYaw >= 45 && mYaw < 135)) {
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
    
    if ((mYaw >= 135 && mYaw <= 180) || (mYaw < -135 && mYaw >= -180)) {
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

    if (mYaw >= -135 && mYaw < -45) {
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

/*
    if (pov == 45.0) {
      m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBL);
    }

    if (pov == 135.0) {
      m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFL);
    }

    if (pov == 225.0) {
      m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFR);
    }

    if (pov == 315.0 ) {
      m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBR);
    }
    */

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
