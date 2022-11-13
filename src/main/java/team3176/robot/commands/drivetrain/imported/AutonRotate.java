// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.drivetrain.imported;

import java.sql.Driver;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;

public class AutonRotate extends CommandBase {

  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private double rotationSpeed;
  private double degrees;
  private double initialAngle;
  private double goal;
  private double currentAngle;

  /** To go in the negative direction put a negative rotational speed and positive degrees. */
  public AutonRotate(double rot, double degrees) {
    addRequirements(m_Drivetrain);
    rotationSpeed = rot;
    this.degrees = degrees;
  }

  @Override
  public void initialize() {
    m_coordSys.setCoordType(coordType.ROBOT_CENTRIC);
    //initialAngle = -SwerveSubsystem.getNavxAngle_inDegrees();
    initialAngle = m_gyro.getNavxAngle_inDegrees();
    //rotation = Math.copySign(rotation, degrees);
    
    //goal = initialAngle + degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Drivetrain.drive(0,0,rotationSpeed);
    currentAngle = -m_gyro.getNavxAngle_inDegrees();
    SmartDashboard.putNumber("Rotate.initialAngle", initialAngle);
    SmartDashboard.putNumber("Rotate.currentAngle", currentAngle);
    // SmartDashboard.putNumber("goal", goal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  /*public boolean isFinished() {
    if ((degrees >= 0 && currentAngle >= goal) || (degrees < 0 && currentAngle <= goal)) {
      return true;
    }
    return false;
  }*/
  public boolean isFinished() {
    if(rotationSpeed > 0){
      if(m_gyro.getNavxAngle_inDegrees() >= initialAngle + degrees ){
        m_Drivetrain.drive(0,0,0);
        return true;
      }
    }
    if(rotationSpeed < 0){
      if(m_gyro.getNavxAngle_inDegrees() <= initialAngle + -degrees ){
        m_Drivetrain.drive(0,0,0);
        return true;
      }
    }
    return false;
  }
}
