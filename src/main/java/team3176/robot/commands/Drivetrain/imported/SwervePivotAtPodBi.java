/*
/*
 
   ____  _            _           _     ____           _ 
  |  _ \(_)_   _____ | |_    __ _| |_  |  _ \ ___   __| |
  | |_) | \ \ / / _ \| __|  / _` | __| | |_) / _ \ / _` |
  |  __/| |\ V / (_) | |_  | (_| | |_  |  __/ (_) | (_| |
  |_|   |_| \_/ \___/ \__|  \__,_|\__| |_|   \___/ \__,_|
                                                         
 
*/
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

public class SwervePivotAtPodBi extends CommandBase {
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private DoubleSupplier forwardCommand, strafeCommand, spinCommand;
  private Double hatPov, pov;
  private double yaw, m_yaw;

  private double radianOffset;

  public SwervePivotAtPodBi(DoubleSupplier forwardCommand, DoubleSupplier strafeCommand, DoubleSupplier spinCommand, Double hatPov) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;
    this.hatPov = hatPov;
    this.pov = this.hatPov;
    addRequirements(m_SwerveSubsystem, m_Gyro);
  }

  @Override
  public void initialize() {
    //m_yaw = 0;
    m_yaw = m_Gyro.getYaw();
    yaw = m_Gyro.getYaw();

    //if (yaw < 0) { m_yaw = yaw % -180.0; } else { m_yaw = yaw % 180;}

    /*
    if ((m_yaw >= 45 && m_yaw < 135)) {
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
    
    if ((m_yaw >= 135 && m_yaw <= 180) || (m_yaw < -135 && m_yaw >= -180)) {
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

    if (m_yaw >= -135 && m_yaw < -45) {
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
    */
  }

  @Override
  public void execute() {


    if (pov == 315 || pov == 0 || pov == 45) {
      if ((m_yaw >= -45 && m_yaw <=0) || (m_yaw >= 0 && m_yaw <= 44 )) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBRBL);
      }
      if (m_yaw >= 45 && m_yaw <= 134) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFRBR);
      } 
      if ((m_yaw >= 135 && m_yaw <=180) || (m_yaw >= -180 && m_yaw <= -136)) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFLFR);
      } 
      if (m_yaw <= -135 && m_yaw >=-46) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBLFL);
      } 
    }
      
    if (pov == 135 || pov == 180 || pov == 225) {
      if ((m_yaw >= -45 && m_yaw <=0) || (m_yaw >= 0 && m_yaw <= 44 )) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFRFL);
      }
      if (m_yaw >= 45 && m_yaw <= 134) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTFLBL);
      } 
      if ((m_yaw >= 135 && m_yaw <=180) || (m_yaw >= -180 && m_yaw <= -136)) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBLBR);
      } 
      if (m_yaw <= -135 && m_yaw >=-46) {
        m_SwerveSubsystem.setDriveMode(driveMode.PIVOTBRFR);
      } 
    }
      

    m_SwerveSubsystem.drive(forwardCommand.getAsDouble() * SwerveSubsystemConstants.MAX_ACCEL_FEET_PER_SECOND, strafeCommand.getAsDouble() * SwerveSubsystemConstants.MAX_ACCEL_FEET_PER_SECOND, spinCommand.getAsDouble() * 25 /* inches */); //}
  }

  @Override
  public boolean isFinished() { return false; }

  @Override
  public void end(boolean interrupted) {
    //if(wasFieldCentric) {

    m_SwerveSubsystem.setDriveMode(driveMode.DRIVE);
  }
}
