// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;


public class Odometry3176 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Odometry3176() {
    private SwerveDriveOdometry odometry;
    
    // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);
    //odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, getNavxAngle_inRadians_asRotation2d()); 
    }
  }
  
  
      
  public Pose2d getCurrentPose() {
    // SmartDashboard.putNumber("odometry X", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("odometry Y", odometry.getPoseMeters().getY());
    return odometry.getPoseMeters() ; //Does this work?
  }
  
  public void resetOdometry(Pose2d pose) {
    //odometry.resetPosition(pose, gyro.getRotation2d().times(1));  //Not sure, gyroAngle);a   // <-- Note getRotation2d is continuous, ie 360+1=361 not 0 or -359 
    odometry.resetPosition(pose, getNavxAngle_inRadians_asRotation2d());  
  }
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
