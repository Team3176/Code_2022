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
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TrapezoidRotate extends CommandBase {
  /** Creates a new TrapezoidDrive. */
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Timer timer = new Timer();
  private TrapezoidProfile profile;
  private double theta;
  private double distanceX;
  private double distanceY;
  private double rotation_distance, direction;
  private double botCircumferance = 12.315;  //feet
  private double botCircumferancePer1Degree;

  public TrapezoidRotate(double direction, double angle) { //TODO: FIIIIIIIIXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxx
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
    //this.distanceX = distanceX;
    //this.distanceY = distanceY;
    this.botCircumferancePer1Degree = botCircumferance / 360;
    m_CoordSys.setCoordTypeToRobotCentric();
    //double hyp = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    //this.theta = Math.atan(distanceY / distanceX);
    this.direction = direction;
    this.rotation_distance = angle * botCircumferancePer1Degree;
    profile = new TrapezoidProfile
    (new TrapezoidProfile.Constraints(14, 12), new TrapezoidProfile.State(rotation_distance, 0), new TrapezoidProfile.State(0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CoordSys.setCoordType(coordType.ROBOT_CENTRIC);
    //m_gyro.setSpinLockAngle();
    //m_gyro.setSpinLock(true);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State setPoint = profile.calculate(timer.get());
    /// double velocityX = setPoint.velocity * Math.cos(theta);
    //double velocityY = setPoint.velocity * Math.sin(theta);
    double spinCommand = setPoint.velocity * rotation_distance;
    if (this.direction > 0) {spinCommand *= -1;}
    m_Drivetrain.drive(0, 0, spinCommand);

    // System.out.println("X: " + distanceX + ", Y: " + distanceY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gyro.setSpinLockToOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }
}
