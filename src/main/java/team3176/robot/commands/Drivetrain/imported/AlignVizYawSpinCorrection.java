// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Drivetrain.imported;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.Feeder;
import team3176.robot.subsystems.Vision;
import team3176.robot.subsystems.drivetrain.CoordSys;


/**
 * AutoAlign: A simplistic command class to retrieve the x-angle (tx) formed
 * by the LL crosshairs, the lens, and the recognized target.  It (tx) is then
 * used to call AutoRotate(tx) to rotate the bot until the angle is within the range 
 * formed by upperTxLimit and lowerTxLimit
 */
public class AlignVizYawSpinCorrection extends SequentialCommandGroup {

  private Drivetrain m_drivetrain = Drivetrain.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();
  
  private double tx, yawError;
  private double upperTxLimit, lowerTxLimit;
  private double kP, minCommand;

  /** Creates a new AutonAlign. */
  public AlignVizYawSpinCorrection() {
    addRequirements(m_drivetrain);
    addRequirements(m_Vision);
    //addRequirements(m_Transfer);
  }

  @Override
  public void initialize() {
    // m_drivetrain.setCoordType(coordType.ROBOT_CENTRIC);
    m_coordSys.setCoordType(coordType.FIELD_CENTRIC);
    //m_Vision.setPipeline(1);
    //m_Vision.turnLEDsOn();
    this.kP = -0.01;
    this.minCommand = 0.001;
    this.upperTxLimit = 3;
    this.lowerTxLimit = -3;
    m_drivetrain.drive(0,0,0);
    m_Vision.setVisionSpinCorrectionOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.yawError = m_Vision.tx.getDouble(0);
    m_drivetrain.drive(0,0,0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.stopMotors();
      m_Vision.setVisionSpinCorrectionOff();
      //m_Vision.setPipeline(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.yawError >= lowerTxLimit && this.yawError <= upperTxLimit) {
      return true;
    }
    return false;
  }
}
