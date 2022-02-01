// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

public class Nav extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private SwerveDriveOdometry odometry;
  private static Nav instance = new Nav();
  Gyro3176 m_Gyro3176 = Gyro3176.getInstance();
  CoordSys m_CoordSys = CoordSys.getInstance();
  Odometry3176 m_Odometry3176 = Odometry3176.getInstance();
  private double currentAngle;  
  
  public Nav() {
    /* 
    if (this.isSpinLocked && !isOrbiting()) {
      this.spinCommand = -spinLockPID.returnOutput(getNavxAngle_inRadians(), spinLockAngle);
      // this.spinCommand = spinLockPID.calculate(getNavxAngle(), spinLockAngle);
    }
    */

  }

  public double getCurrentAngleFromGyro() {
    this.currentAngle = m_Gyro3176.getCurrentAngle();
    return this.currentAngle;
  }
    
  
      
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static Nav getInstance() {
    return instance;
  }
}
