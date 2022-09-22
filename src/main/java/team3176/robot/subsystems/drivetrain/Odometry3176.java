// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import team3176.robot.subsystems.SwerveSubsystem.SwervePod2022;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.*;
import team3176.robot.constants.SwerveSubsystemConstants; 
import edu.wpi.first.math.geometry.Translation2d;

public class Odometry3176 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SwerveDriveOdometry m_odometry;
  private static Odometry3176 instance = new Odometry3176();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();

  private Translation2d m_podFRLocation; 
  private Translation2d m_podFLLocation; 
  private Translation2d m_podBLLocation; 
  private Translation2d m_podBRLocation; 
  private SwerveDriveKinematics m_kinematics; 
  private ChassisSpeeds m_chassisSpeeds; 
  private double forwardVelocity;
  private double strafeVelocity;
  private double spinVelocity;
  private Pose2d startingPos;
  private Pose2d m_pose;
  

  public Odometry3176() {
    // Locations for the swerve drive modules relative to the robot center.
    m_podFRLocation = new Translation2d(SwerveSubsystemConstants.LENGTH / 2.0 , -SwerveSubsystemConstants.WIDTH / 2.0);
    m_podFLLocation = new Translation2d(SwerveSubsystemConstants.LENGTH / 2.0, SwerveSubsystemConstants.WIDTH / 2.0);
    m_podBLLocation = new Translation2d(-SwerveSubsystemConstants.LENGTH / 2.0, SwerveSubsystemConstants.WIDTH / 2.0);
    m_podBRLocation = new Translation2d(-SwerveSubsystemConstants.LENGTH / 2.0, -SwerveSubsystemConstants.WIDTH / 2.0);

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(m_podFRLocation, m_podFLLocation, m_podBLLocation, m_podBRLocation);
  
    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    // FYI: Pose2d(x-coord, y-coord) per WPILib API.  So i dunno WTF the above description really means about long and short edges of field.
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_Gyro.getHeading(), new Pose2d(5.0, 13.5, new Rotation2d()));

  }

  public Pose2d getCurrentPose() {
    SmartDashboard.putNumber("odometry X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("odometry Y", m_odometry.getPoseMeters().getY());
    return m_odometry.getPoseMeters() ; //Does this work?
  }
  
  public void resetOdometry(Pose2d pose) {
    //odometry.resetPosition(pose, gyro.getRotation2d().times(1));  //Not sure, gyroAngle);a   // <-- Note getRotation2d is continuous, ie 360+1=361 not 0 or -359 
    m_odometry.resetPosition(pose, m_Gyro.getNavxAngle_inRadians_asRotation2d());  
  }
  
  public void chassisSpeedConversion() {

    
    
    
    // Convert to chassis speeds
    m_chassisSpeeds = m_kinematics.toChassisSpeeds(m_SwerveSubsystem.getPodState(0), m_SwerveSubsystem.getPodState(1),m_SwerveSubsystem.getPodState(2), m_SwerveSubsystem.getPodState(3));

    // Get individual speeds
    forwardVelocity = m_chassisSpeeds.vxMetersPerSecond;
    strafeVelocity = m_chassisSpeeds.vxMetersPerSecond;
    spinVelocity = m_chassisSpeeds.omegaRadiansPerSecond;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    m_pose = m_odometry.update(m_Gyro.getHeading(),m_SwerveSubsystem.getPodState(0), m_SwerveSubsystem.getPodState(1),m_SwerveSubsystem.getPodState(2), m_SwerveSubsystem.getPodState(3));
    getCurrentPose(); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Odometry3176 getInstance() {
    return instance;
  }
}
