// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
//import edu.wpi.first.util.sendable.Sendable;
//import team3176.robot.subsystems.drivetrain.Gyro3176;

public class CoordSys extends SubsystemBase {
  private static CoordSys instance = new CoordSys(); 
  private Gyro3176 m_Gyro = Gyro3176.getInstance();
  


  public enum coordType {
    FIELD_CENTRIC, ROBOT_CENTRIC
  }
  private coordType currentCoordType;
  private double fieldCentricOffset = 0.0;


  /** Creates a new ExampleSubsystem. */
  public CoordSys() {
    this.currentCoordType = coordType.ROBOT_CENTRIC;
  }

  public void setCoordTypeToFieldCentric() {
    this.currentCoordType = coordType.FIELD_CENTRIC;
    System.out.println("COORDSYS: FIELD_CENTRIC ############################");
  }

  public void setCoordTypeToRobotCentric() {
    this.currentCoordType = coordType.ROBOT_CENTRIC;
    System.out.println("COORDSYS: ROBOT_CENTRIC ############################");

  }

  public void setCoordType(coordType wantedType) {
    currentCoordType = wantedType;
  }

  public coordType getCurrentCoordType() {
    return currentCoordType;
  }

  public boolean isFieldCentric() {
    if (currentCoordType == coordType.FIELD_CENTRIC) {
      return true;
     } else {
       return false;
     }
  }

  public boolean isRobotCentric() {
    if (currentCoordType == coordType.ROBOT_CENTRIC) {
      return true;
     } else {
       return false;
     }
  }
  
  public void setCurrentCoordType(String CoordType) {
    if (CoordType.equals("FIELD_CENTRIC")) {
      currentCoordType = coordType.FIELD_CENTRIC;
    }
    if (CoordType.equals("ROBOT_CENTRIC")) {
      currentCoordType = coordType.ROBOT_CENTRIC;
    }
  }
 
  public void setFieldCentricOffset() {
    fieldCentricOffset = m_Gyro.getNavxAngle_inRadians();
    // SmartDashboard.putNumber("value in Drivetrain", getNavxAngle_inRadians());
  }  

  public double getFieldCentricOffset() { 
    return fieldCentricOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static CoordSys getInstance() {
    return instance;
  }
}
