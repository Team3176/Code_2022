// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import team3176.robot.subsystems.drivetrain.SwervePod2022;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.*;
import team3176.robot.constants.DrivetrainConstants; 
import edu.wpi.first.math.geometry.Translation2d;

public class Odometry3176 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SwerveDriveOdometry odometry;
  private static Odometry3176 instance = new Odometry3176();
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();

  private Translation2d m_pod0Location; 
  private Translation2d m_pod1Location; 
  private Translation2d m_pod2Location; 
  private Translation2d m_pod3Location; 
  private SwerveDriveKinematics m_kinematics; 
  private SwerveModuleState m_pod0State, m_pod1State, m_pod2State, m_pod3State;
  private ChassisSpeeds m_chassisSpeeds; 
  private double forwardVelocity;
  private double strafeVelocity;
  private double spinVelocity;
  private SwerveDriveOdometry m_odometry; 
  // private Pose2d startingPos;
  private Pose2d m_pose;
  

  public Odometry3176() {
    // Locations for the swerve drive modules relative to the robot center.
    m_pod0Location = new Translation2d(DrivetrainConstants.POD0_LOCATION_X, DrivetrainConstants.POD0_LOCATION_Y);
    m_pod1Location = new Translation2d(DrivetrainConstants.POD1_LOCATION_X, DrivetrainConstants.POD1_LOCATION_Y);
    m_pod2Location = new Translation2d(DrivetrainConstants.POD2_LOCATION_X, DrivetrainConstants.POD2_LOCATION_Y);
    m_pod3Location = new Translation2d(DrivetrainConstants.POD3_LOCATION_X, DrivetrainConstants.POD3_LOCATION_Y);

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(m_pod0Location, m_pod1Location, m_pod2Location, m_pod3Location);
  
    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    // FYI: Pose2d(x-coord, y-coord) per WPILib API.  So i dunno WTF the above description really means about long and short edges of field.
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_Gyro.getNavxAngle_inRadians_asRotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));
    // TODO: Check if we should be doing m_Gyro.getNavxAngle_inRadians_asRotation2d or if it should be in Degrees as a Rotation2d object.
    
    // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);
    //odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, getNavxAngle_inRadians_asRotation2d()); 
  }

  public Pose2d getCurrentPose() {
    // SmartDashboard.putNumber("odometry X", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("odometry Y", odometry.getPoseMeters().getY());
    return odometry.getPoseMeters() ; //Does this work?
  }
  
  public void resetOdometry(Pose2d pose) {
    //odometry.resetPosition(pose, gyro.getRotation2d().times(1));  //Not sure, gyroAngle);a   // <-- Note getRotation2d is continuous, ie 360+1=361 not 0 or -359 
    odometry.resetPosition(pose, m_Gyro.getNavxAngle_inRadians_asRotation2d());  
  }
  
  public void chassisSpeedConversion() {

    double pod0Velocity = m_Drivetrain.getPodVelocity(0);
    double pod1Velocity = m_Drivetrain.getPodVelocity(1);
    double pod2Velocity = m_Drivetrain.getPodVelocity(2);
    double pod3Velocity = m_Drivetrain.getPodVelocity(3);

    double pod0AzimuthInDegrees = m_Drivetrain.getPodAzimuth(0);
    double pod1AzimuthInDegrees = m_Drivetrain.getPodAzimuth(1);
    double pod2AzimuthInDegrees = m_Drivetrain.getPodAzimuth(2);
    double pod3AzimuthInDegrees = m_Drivetrain.getPodAzimuth(3);


    m_pod0State = new SwerveModuleState(pod0Velocity, Rotation2d.fromDegrees(pod0AzimuthInDegrees));
    m_pod1State = new SwerveModuleState(pod1Velocity, Rotation2d.fromDegrees(pod1AzimuthInDegrees));
    m_pod2State = new SwerveModuleState(pod2Velocity, Rotation2d.fromDegrees(pod2AzimuthInDegrees));
    m_pod3State = new SwerveModuleState(pod3Velocity, Rotation2d.fromDegrees(pod3AzimuthInDegrees));
    
    
    // Convert to chassis speeds
    m_chassisSpeeds = m_kinematics.toChassisSpeeds(m_pod0State, m_pod1State, m_pod2State, m_pod3State);

    // Get individual speeds
    forwardVelocity = m_chassisSpeeds.vxMetersPerSecond;
    strafeVelocity = m_chassisSpeeds.vxMetersPerSecond;
    spinVelocity = m_chassisSpeeds.omegaRadiansPerSecond;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    m_pose = m_odometry.update(m_Gyro.getNavxAngle_inRadians_asRotation2d(),m_pod0State); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Odometry3176 getInstance() {
    return instance;
  }
}
