// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.VisionConstants;
public class Vision extends SubsystemBase {
  
  private static Vision instance = new Vision();

  public NetworkTableInstance tableInstance;
  public NetworkTable limelightTable;
  public NetworkTableEntry tv;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tshort;
  public NetworkTableEntry tlong;
  public NetworkTableEntry thor;
  public NetworkTableEntry tvert;
  public NetworkTableEntry tcornxy;
  private NetworkTableEntry tl;
  private NetworkTableEntry pipeline;
  private NetworkTableEntry camMode;
  private NetworkTableEntry ledMode;
    
  private double activePipeline = 1;
  private double startTime;

  private double deltaXCam;
  private double radius;

  // initializing variables for kinematic calculations
  private final double gravity = -9.81; // m/s^2
  private double deltaX; // m
  private double deltaY; // m
  private double initialVelocity; // m/s
  private double[] initialAngle = {45, 50, 55, 60, 65, 70, 75, 80, 85, 90}; // deg from horizontal
  private double finalAngle; // radians
  private double xVelocity; // m/s
  private double initialYVelocity; // m/s
  private double finalYVelocity; // m/s
  private double time; // seconds

  private ArrayList<Double> tcornx;
  private ArrayList<Double> tcorny;

  //private int ballLocation = -999; // -999=no ball detected, 0=ball to left, 1=ball exactly 0 degrees forward, 2=ball to right
  //private double ballDegrees = -999; // degrees away from Limelight where ball is located. Positive = to left. Negative = to right. Zero = straight ahead.

  /**
   * Creates the default references for VisionClient, specifically for Limelight values
   */
  public Vision(){
    tableInstance = NetworkTableInstance.getDefault();
    limelightTable = tableInstance.getTable("limelight");
    updateVisionData();

    limelightTable.getEntry("pipeline").setNumber(activePipeline);
  }

  public static Vision getInstance(){
    return instance;
  }

  /**
   * Can be called to force update of VisionClient data structure
   */
  public void updateVisionData(){
    tv = limelightTable.getEntry("tv");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    tshort = limelightTable.getEntry("tshort");
    tlong = limelightTable.getEntry("tlong");
    thor = limelightTable.getEntry("thor");
    tvert = limelightTable.getEntry("tvert");
    tcornxy = limelightTable.getEntry("tcornxy");
    tl = limelightTable.getEntry("tl");
    pipeline = limelightTable.getEntry("pipeline");
    camMode = limelightTable.getEntry("camMode");
    ledMode = limelightTable.getEntry("ledMode");
    activePipeline = pipeline.getDouble(0);
  }

  public void targetRecogControlLoop(){
    // used to calculate latency
    startTime = Timer.getFPGATimestamp();

    updateVisionData();

    SmartDashboard.putNumber("Info", tcornxy.getDoubleArray(new double[0]).length);
    if(tcornxy.getDoubleArray(new double[0]).length != 8){
      SmartDashboard.putBoolean("Has Run?", true);
      return;
    }

    separateCornArray();

    deltaXCam = findDeltaX();

    calculateTargetDistance();

    // get the initial velocity and angle of ball
    findInitialAngleAndVelocity((int) Math.ceil(initialAngle.length / 2));

    publishAllData();

    SmartDashboard.putNumber("Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
  }

  public void separateCornArray(){
    tcornx.clear();
    tcorny.clear();
    for(int i = 0; i < 5; i++){
      tcornx.add(limelightTable.getEntry("x" + i).getDouble(-999.0));
      tcorny.add(limelightTable.getEntry("y" + i).getDouble(-999.0));
    }
    tcornx.remove(-999.0);
    tcorny.remove(-999.0);
  }

  public double findDeltaX(){
    Double[] tempCorn = (Double[]) tcornx.toArray();
    Arrays.sort(tempCorn);
    return tempCorn[tempCorn.length - 1] - tempCorn[0];
  }

  public void calculateTargetDistance(){
    double rawDistance = VisionConstants.VISION_CONSTANT / deltaXCam;
    deltaX = rawDistance * Math.cos(VisionConstants.cameraAngle * VisionConstants.DEG2RAD) + 10 * VisionConstants.INCHES2METERS;
    deltaY = rawDistance * Math.sin(VisionConstants.cameraAngle * VisionConstants.DEG2RAD);
    SmartDashboard.putNumber("DeltaX", deltaX);
    SmartDashboard.putNumber("DeltaY", deltaY);
  }

  public double[] findInitialAngleAndVelocity(int angleIdx){
    if(angleIdx >= initialAngle.length || angleIdx < 0){
      return null;
    }
    
    findInitialVelocity(angleIdx);
    
    // Haha, get it? Because this variable "doublechecks" the result. Programming jokes are the best. 
    double check = Math.sqrt(Math.pow(initialVelocity * Math.sin(initialAngle[angleIdx]), 2) + 2 * -gravity * deltaY);
    
    if(check > 0){
      time = (check - initialVelocity) / -gravity;
      double fullCalculatedDistance = initialVelocity * Math.cos(initialAngle[angleIdx]) * time;
      if(fullCalculatedDistance > deltaX){
        return findInitialAngleAndVelocity(angleIdx + 1);
      } else{
        return findInitialAngleAndVelocity(angleIdx - 1);
      }
    }
    
    solveOtherVariablesFromVelocity();
    
    double[] arrayToSend = {initialVelocity, initialAngle[angleIdx]};
    return arrayToSend;
  }
  
  private void findInitialVelocity(int angleIdx){
    double term1 = deltaX / (Math.cos(initialAngle[angleIdx]));
    double term2 = gravity / (2 * (deltaX * Math.tan(initialAngle[angleIdx]) - deltaY));
    initialVelocity = term1 * Math.sqrt(term2);
  }
  
  private void solveOtherVariablesFromVelocity(){

  }

  public void setVisionProcessing(boolean imageProcessing){
    if(imageProcessing){
      camMode.setNumber(0);
    } else{
      camMode.setNumber(1);
    }
  }

  public boolean getVisionProcessing(){
    return camMode.getNumber(0).equals(0.0);
  }

  public void setActivePipeline(double newPipelineNum){
    if((int) newPipelineNum >= 0 || (int) newPipelineNum <= 5){
      activePipeline = newPipelineNum;
      pipeline.setNumber(newPipelineNum);
    } else{
      System.out.println("Invalid Pipeline Requested, No Change Was Made");
    }
  }

  private void publishAllData(){
    SmartDashboard.putNumber("initialVelocity", initialVelocity);

    SmartDashboard.putBoolean("Has Targets", (tv.getDouble(0) == 1));
    SmartDashboard.putNumber("tshort", tshort.getDouble(0));
    SmartDashboard.putNumber("tvert", tvert.getDouble(0));

    double numTargets = tcornx.size();
    SmartDashboard.putNumber("Number of Targets", numTargets);

    SmartDashboard.putNumber("Radius", radius);
    SmartDashboard.putNumber("Horizontal Distance", deltaX);
    SmartDashboard.putNumber("Vertical Distance", deltaY);

    SmartDashboard.putNumber("Distance According to Camera", deltaXCam);

    SmartDashboard.putNumber("Approx. Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
  }

  public double getCurrentPipeline(){
    return pipeline.getDouble(0);
  }

  public void switchLEDs(LEDState newLEDState){
    if(newLEDState == LEDState.OFF){
      ledMode.setNumber(1);
    } else if(newLEDState == LEDState.ON){
      ledMode.setNumber(2);
    } else if(newLEDState == LEDState.BLINK){
      ledMode.setNumber(3);
    } else{
      System.out.println("Invalid LED State Requested, No Change Made");
    }
  }

  public enum LEDState {
    OFF, ON, BLINK, NULL
  }

  ArrayList<Double> testValues = new ArrayList<Double>();

  /*public void averageMeasurements(double newValue){
    testValues.add(newValue);
    double total = 0;
    for(double value : testValues) {
      total += value;
    }
    SmartDashboard.putNumber("Average Distance", total / testValues.size());
    System.out.println("DONE!");
  }*/
}
