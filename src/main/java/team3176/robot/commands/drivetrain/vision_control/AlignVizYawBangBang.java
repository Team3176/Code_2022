// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain.vision_control;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.vision.Vision;
import team3176.robot.subsystems.drivetrain.CoordSys;


/**
 * AutoAlign: A simplistic command class to retrieve the x-angle (tx) formed
 * by the LL crosshairs, the lens, and the recognized target.  It (tx) is then
 * used to call AutoRotate(tx) to rotate the bot until the angle is within the range 
 * formed by upperTxLimit and lowerTxLimit
 */
public class AlignVizYawBangBang extends CommandBase {

    private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();
  //private Vision m_Vision = Vision.getInstance();
  private double tx;
  private double upperTxLimit, lowerTxLimit;

  /** Creates a new AutonAlign. */
  public AlignVizYawBangBang() {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    // m_SwerveSubsystem.setCoordType(coordType.ROBOT_CENTRIC);
    m_coordSys.setCoordType(coordType.FIELD_CENTRIC);
   // m_Vision.turnLEDsOn();
    this.upperTxLimit = 1;
    this.lowerTxLimit = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this.tx =  m_Vision.getTx();
    //new AutonRotate(.1, tx);
    m_Drivetrain.drive(0, 0, Math.copySign(.05, -tx));
    // SmartDashboard.putNumber("AlignVizYawBangBang.tx", -tx);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_Drivetrain.drive(0,0,0);
      //m_Vision.turnLEDsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.tx >= lowerTxLimit && this.tx <= upperTxLimit){
      return true;
    }
    return false;
  }
}
