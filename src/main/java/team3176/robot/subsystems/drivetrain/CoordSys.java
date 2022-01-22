// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

public class CoordSys extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CoordSys() {}
  
  public coordType getCurrentCoordType() {
    return currentCoordType;
  }
  
  public coordType setCurrentCoordType(String CoordType) {
    if (CoordType.equals("FIELD_CENTRIC")) {
      currentCoordType = coordType.FIELD_CENTRIC;
    }
    if (CoordType.equals("ROBOT_CENTRIC")) {
      currentCoordType = coordType.ROBOT_CENTRIC;
    }
  }
  
  public void setFieldCentricOffset() {
    fieldCentricOffset = getNavxAngle_inRadians();
    // SmartDashboard.putNumber("value in Drivetrain", getNavxAngle_inRadians());
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
