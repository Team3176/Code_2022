// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.kauailabs.navx.frc.AHRS;

public class Gyro3176 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);
    private AHRS gyro;
    private double gyroOffset = 0;

    private double lastGyroClock;
  
    public Gyro3176() {
      
      // Instantiating the gyro
      gyro = new AHRS(SPI.Port.kMXP);
      gyro.reset();
      // gyro.setAngleAdjustment(90.0);
      // gyroUpdateOffset();
      updateNavxAngle();
      // odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS,
      // gyro.getRotation2d()); // <<-- getRotation2d is continuous. ie 360+1=361 not
      // 0 or -361. gyro.getRotation2d() uses NWU Axis Convention

      //Is continuous. ie 360+1=361 not m0 or -361. getNavxAngle_asRotation2d() should be same Axis Convention as Teleop, I believe.
      //odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, getNavxAngle_inRadians_asRotation2d()); 
    }
  }
  
  public void driveGyro() {
    
    updateNavxAngle();
    // SmartDashboard.putNumber("Drive updated currentAngle Degrees",
    // (this.currentAngle * 180/Math.PI));
    // SmartDashboard.putString("Drive currentCoordType",
    // currentCoordType.toString()); 
    
    if (this.isSpinLocked && !isOrbiting()) {
      this.spinCommand = -spinLockPID.returnOutput(getNavxAngle_inRadians(), spinLockAngle);
      // this.spinCommand = spinLockPID.calculate(getNavxAngle(), spinLockAngle);
    }
   }

  public double getNavxAngle_inDegrees() {
    return (gyro.getAngle() + DrivetrainConstants.GYRO_COORDSYS_ROTATIONAL_OFFSET + this.gyroOffset);
  }
  
    private double getNavxAngle_inRadians() {
    return (Units.degreesToRadians(getNavxAngle_inDegrees()));
  }
  
  public Rotation2d getNavxAngle_inRadians_asRotation2d() {
    Rotation2d angle = new Rotation2d(getNavxAngle_inRadians());
    return angle;
  }
  
   private void updateNavxAngle() {
    // -pi to pi; 0 = straight
    this.currentAngle = (((Units.degreesToRadians(getNavxAngle_inDegrees()))) % (2 * Math.PI));
    // gyro.getNavxAngle is returned in degrees.
    // Then converted to radians (ie *(Math.PI/180)).
    // And finally, it's modulus against 2pi is taken and returned as currentAngle.
  }
  
  public void gyroUpdateOffset() {
    this.gyroOffset = (getNavxAngle_inDegrees());
  }
  
  public void resetGyro() {
    gyro.reset();
  }
  
  public double getGyroAngle() {
    return getNavxAngle_inDegrees();
  }
  
  public double getCurrentAngle() {
    updateNavxAngle();
    return this.currentAngle;
  }
  
  public void setSpinLockAngle() {
    spinLockAngle = getNavxAngle_inRadians();
  }
      
  public double getCurrentAngle() {
    updateNavxAngle();
    return this.currentAngle;
  }
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    public double getHeading() {
      // SmartDashboard.putNumber("Drivetrain.getHeading_as_gyro.getRotation2d.getDegrees()", gyro.getRotation2d().getDegrees());
      // SmartDashboard.putNumber("Drivetrain.getHeading_as_getNavxAngle_inDegrees()", getNavxAngle_inDegrees());
      //return gyro.getRotation2d().getRadians() ; //+ Math.PI/2;
      return getNavxAngle_inRadians() ; //+ Math.PI/2;
    }  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
