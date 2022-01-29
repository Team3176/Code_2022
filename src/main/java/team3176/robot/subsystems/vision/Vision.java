// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.VisionClient;
public class Vision extends SubsystemBase {

  public static VisionClient mVisionClient;
  
  public Vision() {
    mVisionClient = new VisionClient();
    mVisionClient.setActivePipeline(2);
  }

  @Override
  public void periodic() {}

  public void SetVisionProcessing(boolean mode){
    mVisionClient.setCameraMode(mode);
  }

  public boolean getVisionProcessing(){
    return mVisionClient.getCameraMode();
  }

  public void changePipeline(double newPipeline){
    mVisionClient.setActivePipeline(newPipeline);
  }

  public void publishAllData(){
    mVisionClient.publishInitialTheta();
    mVisionClient.publishPrelimTargetRecogData();
    mVisionClient.publishTargetRecogDistances();
  }

  public double getCurrentPipeline(){
    return mVisionClient.getActivePipeline();
  }

  public void calculateDistance(){
    mVisionClient.calcTargetRecogDistances();
    SmartDashboard.putNumberArray("tcornx", mVisionClient.tcornx.getDoubleArray(new double[1]));
  }

  @Override
  public void simulationPeriodic() {}
}
