// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutonRotatePidGyro extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private TrapezoidProfile profile;
  private double botCircumferance = 12.315;  //feet
  private double initial_yaw;
  private double yaw_diff;
  private double yaw_TrueSetpoint;

  /**
   * 
   * @param direction  
   * @param angle  
   */
  public AutonRotatePidGyro(double yaw_setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    addRequirements(m_Gyro);
    m_CoordSys.setCoordTypeToFieldCentric();
    m_CoordSys.setCoordTypeToRobotCentric();
    this.yaw_diff = yaw_setpoint;
    this.yaw_TrueSetpoint = this.initial_yaw + yaw_setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initial_yaw = m_Gyro.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double smallnum = Math.pow(10,-10);
    //m_Drivetrain.drive(smallnum, smallnum, smallnum);
    //m_Drivetrain.drive(0,0,0);
    m_Drivetrain.stopMotors();
    //System.out.println("######################################################################################################################     TrapRot.end()");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return profile.isFinished(timer.get());
    return false;
  }
}
