// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import team3176.robot.constants.DrivetrainConstants;
import edu.wpi.first.math.util.Units;
import team3176.robot.util.God.*;

public class Gyro3176 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static Gyro3176 instance = new Gyro3176(); 
  // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);
  private AHRS gyro;
  private double gyroOffset_in_Degrees = 0;
  
  private double currentAngle;
  private double lastAngle;


  private double lastGyroClock;

  private boolean isSpinLocked;
  private double spinLockAngle;

  private PID3176 spinLockPID;
  private MedianFilter angleAvgRollingWindow;
  
  public Gyro3176() {
      
    // Instantiating the gyro
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    angleAvgRollingWindow = new MedianFilter(5);
    // gyro.setAngleAdjustment(90.0);
    // gyroUpdateOffset();
    updateNavxAngle();
    // odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS,
    // gyro.getRotation2d()); // <<-- getRotation2d is continuous. ie 360+1=361 not
    // 0 or -361. gyro.getRotation2d() uses NWU Axis Convention

    //Is continuous. ie 360+1=361 not m0 or -361. getNavxAngle_asRotation2d() should be same Axis Convention as Teleop, I believe.
    //odometry = new SwerveDriveOdometry(DrivetrainConstants.DRIVE_KINEMATICS, getNavxAngle_inRadians_asRotation2d()); 

    isSpinLocked = false;

    spinLockPID = new PID3176(0.15, 0.0, 0.0);

    // sets spin lock angle at startup
    setSpinLockAngle();
  }
  
  
 

  public double getNavxAngle_inDegrees() {
    return (gyro.getAngle() + DrivetrainConstants.GYRO_ROTATIONAL_OFFSET_FOR_RIO_MOUNTING + this.gyroOffset_in_Degrees);
  }
  
  public double getNavxAngle_inRadians() {
    return (Units.degreesToRadians(getNavxAngle_inDegrees()));
  }
  
  public Rotation2d getNavxAngle_inRadians_asRotation2d() {
    Rotation2d angle = new Rotation2d(getNavxAngle_inRadians());
    return angle;
  }

  public double getYaw() {
    return gyro.getYaw();
  }
  
  private void updateNavxAngle() {
    // -pi to pi; 0 = straight
    //this.currentAngle = (((Units.degreesToRadians(getNavxAngle_inDegrees()))) % (2 * Math.PI));
    this.currentAngle = ((getNavxAngle_inRadians()) % (2 * Math.PI));
    // gyro.getNavxAngle is returned in degrees.
    // Then converted to radians (ie *(Math.PI/180)).
    // And finally, it's modulus against 2pi is taken and returned as currentAngle.
  }
  
  public void resetGyroHWValues() {
    gyro.reset();
  }
  
  public double getGyroAngle_inDegrees() {
    return getNavxAngle_inDegrees();
  }
  
  public double getGyroAngle_inRadians() {
    return getNavxAngle_inRadians();
  }

  /**
   * Updates currentAngle by polling the navX, and returns the Yaw in units of radians 
   * @return this.currentAngle
   */
  public double getCurrentChassisYaw() {
    updateNavxAngle();
    return this.currentAngle;
  }

  public double getHeading() {
    // SmartDashboard.putNumber("Drivetrain.getHeading_as_gyro.getRotation2d.getDegrees()", gyro.getRotation2d().getDegrees());
    // SmartDashboard.putNumber("Drivetrain.getHeading_as_getNavxAngle_inDegrees()", getNavxAngle_inDegrees());
    //return gyro.getRotation2d().getRadians() ; //+ Math.PI/2;
    return getNavxAngle_inRadians() ; //+ Math.PI/2;
  } 
  
  public void setSpinLockAngle() {
    spinLockAngle = getNavxAngle_inRadians();
  }
  
  public double getSpinLockAngle() {
    return this.spinLockAngle;
  }
      
      
  public double getSpinLockPIDCalc() {
    double spinCorrection = -spinLockPID.returnOutput(getNavxAngle_inRadians(), spinLockAngle);
    return spinCorrection;
  }

  public boolean getIsSpinLocked() {
    return this.isSpinLocked;
  }

  public void setSpinLockToOn() {
    this.isSpinLocked = true;
    setSpinLockAngle();
  }
  
  public void setSpinLockToOff() {
    this.isSpinLocked = false;
  }
   
  public void toggleSpinLock() {
    this.isSpinLocked = !this.isSpinLocked;
    if (this.isSpinLocked) {
      setSpinLockAngle();
    }
  }
  
  // redundant with above, not sure if this one is used
  public boolean getIsSpinLockTrue() {
    return this.isSpinLocked;
  }
  
  /**
   * @return The degree position of the north direction, according to what the NavX thinks zero is.
   */
  public double getNorth() {
    // this is what the NavX senses as north, and the value reported is the angle the NavX reads as the north direction
    return gyro.getCompassHeading();
  }
  public double getAngleAvgRollingWindow() {
    return angleAvgRollingWindow.calculate(this.getCurrentChassisYaw());
  }
  @Override
  public void periodic() {
    updateNavxAngle();
    // SmartDashboard.putNumber("Drive updated currentAngle Degrees",
    // (this.currentAngle * 180/Math.PI));
    // SmartDashboard.putString("Drive currentCoordType",
    // currentCoordType.toString()); 
    
    /*if (this.isSpinLocked && !isOrbiting()) {
      this.spinCommand = -spinLockPID.returnOutput(getNavxAngle_inRadians(), spinLockAngle);
      // this.spinCommand = spinLockPID.calculate(getNavxAngle(), spinLockAngle);
    }
    */
  }
    // This method will be called once per scheduler run

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Gyro3176 getInstance() {
    return instance;
  }

}
