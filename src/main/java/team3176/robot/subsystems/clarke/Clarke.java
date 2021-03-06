// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.clarke;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.util.God.PID3176;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;



public class Clarke extends SubsystemBase {
  private static Clarke instance = new Clarke();

  String[] def = new String[1];

  /** Creates a new ExampleSubsystem. */

  private DriverStation m_DriverStation;
  public NetworkTableInstance tableInstance;
  public NetworkTable piTable;
  public NetworkTableEntry tv;
  public NetworkTableEntry tx;
  public NetworkTableEntry ty;
  public NetworkTableEntry tshort;
  public NetworkTableEntry tlong;
  public NetworkTableEntry thor;
  public NetworkTableEntry tvert;
  public NetworkTableEntry tcornx;
  private NetworkTableEntry tcorny;
  private NetworkTableEntry tl;
  private NetworkTableEntry pipeline;
  private NetworkTableEntry camMode;
  private NetworkTableEntry ledMode;
  private NetworkTableEntry miny;
  private NetworkTableEntry maxY;
  private NetworkTableEntry width_entry;
  private NetworkTableEntry height_entry;
  private double centX;
  private double center;
  private double width, height;
  private NetworkTableEntry detections;
  private NetworkTableEntry detected_color, detected_xmin, detected_xmax;
    
  private double activePipeline = 1;
  private double startTime;

  private double deltaXCam;
  private double radius;

  // initializing variables for kinematic calculations
  private final double gravity = -9.81; // m/s^2
  private double minX;
  private double maxX;
  private double deltaX; // m
  private double deltaY; // m
  private double[] initialVelocity = {4.0, 3.0, 2.0}; // m/s
  private double initialAngle; // radians
  private double finalAngle; // radians
  private double xVelocity; // m/s
  private double initialYVelocity; // m/s
  private double finalYVelocity; // m/s
  private double time; // seconds
  private PIDController peeEyeDee = new PIDController(.15, 0.0, 0.0);
  private int idxCounter = 0; 
  
  private String returned_color = "bs";
  private String target_color;
  private boolean isClarkeSpinCorrectionOn = false;
  private double clarkeSpinCorrection;

  private ObjectMapper mapper;
  //private int ballLocation = -999; // -999=no ball detected, 0=ball to left, 1=ball exactly 0 degrees forward, 2=ball to right
  //private double ballDegrees = -999; // degrees away from pi where ball is located. Positive = to left. Negative = to right. Zero = straight ahead.

  /**
   * Creates the default references for VisionClient, specifically for pi values
   */
  public Clarke(){
    tableInstance = NetworkTableInstance.getDefault();
    piTable = tableInstance.getTable("ML");
    setTargetColor();

    if (m_DriverStation.getAlliance().equals(Alliance.Red)) {this.target_color = "red";}
    if (m_DriverStation.getAlliance().equals(Alliance.Blue)) {this.target_color = "blue";}
    
    updateVisionData();
    //piTable.getEntry("pipeline").setNumber(activePipeline);
    
  }

  public void setTargetColor() {

  }
  public void findMinAndMax(){
//    String jsonString = detections.getString("{ \"xmax\" : \"0\" , \"xmin\" : \"0\"}"); //the current values are just test cases, make them zero again before comp.
    //String[] returnedArray = jsonString.split(" ");
    boolean minIsSet = false;
    boolean maxIsSet = false;
    this.maxX = 0;
    this.minX = 0;
    //for(String e : returnedArray){
    //  System.out.println(e);
    //}
    /*
    System.out.println(returnedArray);
    for(int i = 0; i < returnedArray.length; i++){
      int ahead = i + 1;
      //System.out.println(returnedArray[i]);
      if(returnedArray[i].equals("\"xmax\":") && !maxIsSet){
        if (returnedArray[ahead].length() > 3){
          returnedArray[ahead] = returnedArray[ahead].substring(0,3);
        }
        else if(returnedArray[ahead].length() == 3){
          returnedArray[ahead] = returnedArray[ahead].substring(0,2);
        }
        else{
          returnedArray[ahead] = returnedArray[ahead].substring(0,1);
        }
        String result = returnedArray[ahead].toString();
        result = result.replaceAll( "[^\\d", "");
        this.maxX = Integer.parseInt(result);
        maxIsSet = true;
      }
      if(returnedArray[i].equals("\"xmin\":") && !minIsSet){
        if (returnedArray[ahead].length() > 3){
          returnedArray[ahead] = returnedArray[ahead].substring(0,3);
        }
        else if(returnedArray[ahead].length() == 3){
          returnedArray[ahead] = returnedArray[ahead].substring(0,2);
        }
        else{
          returnedArray[ahead] = returnedArray[ahead].substring(0,1);
        }
        String result = returnedArray[ahead].toString();
        result = result.replaceAll( "[^\\d", "");
        this.minX = Integer.parseInt(result);
        minIsSet = true;
      }
    }
    */

    //System.out.println("Targeting "+ this.target_color + " and "+ this.returned_color +" found"); 
  }
  public void updateMLData(){ 
    piTable = tableInstance.getTable("ML");
    detections = piTable.getEntry("detections");
    detected_color = piTable.getEntry("color");
    this.returned_color = detected_color.getString("bs");
    detected_xmin = piTable.getEntry("xmin");
    detected_xmax = piTable.getEntry("xmax");
    SmartDashboard.putString("Clark.detected_xmax", detected_xmax.getString(""));
    SmartDashboard.putString("Clark.detected_xmin", detected_xmin.getString(""));
    this.maxX = detected_xmax.getDouble(0);
    this.minX = detected_xmin.getDouble(0);
    SmartDashboard.putNumber("Clark.maxX", this.maxX);
    SmartDashboard.putNumber("Clark.minX", this.minX);
    //System.out.println(this.maxX);
    //System.out.println(this.minX);
    width_entry = piTable.getEntry("width");
    width = width_entry.getDouble(0);
    this.centX = this.minX + ( ( this.maxX -  this.minX)/2);
    //System.out.println(this.centX);
    if (this.centX > width / 2.0) {
      this.center = (this.centX - width/2.0);
    } else
    // if ((this.centX == 0) || (this.centX < ((width/2.0)+5) && this.centX > ((width/2.0)-5))) {  
    if(this.centX == width/2){
      this.center = 0;
    } else     if (this.centX < width / 2.0) {
      this.center = -((width /2) - this.centX);
    }
    /*if ((this.centX == 0) || (this.centX < ((width/2.0)+5) && this.centX > ((width/2.0)-5))) {
      this.center = 0;
    }*/

    
    SmartDashboard.putString("Clarke.color", returned_color);
    SmartDashboard.putNumber("Clarke.centX", this.centX);
    SmartDashboard.putNumber("Clarke.center", this.center);
    
    //String myvalue = detections.getStringArray("detections"); 
  }

  public double getCenter(){
    double returnValue = 0;
    if (this.returned_color == this.target_color) {
      updateMLData();
      findMinAndMax();
      returnValue = this.center;
    } 
    return returnValue;
  } 

  /*
  public void calcCenter(){
    center = 0.099999;
    System.out.println(this.maxX);
    System.out.println(this.minX);
    this.centX = ( this.maxX -  this.minX)/2;
    System.out.println(this.centX);
    this.center = (160 - this.centX - 160/2);
    SmartDashboard.putString("Clarke.color", returned_color);
    SmartDashboard.putNumber("Clarke.centX", this.centX);
    SmartDashboard.putNumber("Clarke.center", this.center);
  } 
  */
  public boolean getIsClarkeSpinCorrectionOn(){
    return this.isClarkeSpinCorrectionOn;
  }

  public void setClarkeSpinCorrection(boolean onOrOff) {
    this.isClarkeSpinCorrectionOn = onOrOff;
    SmartDashboard.putBoolean("ClarkeSpinCorrectionOn", isClarkeSpinCorrectionOn);
  }

  public void setClarkeSpinCorrectionOn(){
    if (this.tv.getBoolean(false)) { 
      setClarkeSpinCorrection(true);
      SmartDashboard.putBoolean("ClarkeSpinCorrectionOn", true);
    }
  }
  
  public void setClarkeSpinCorrectionOff(){
    setClarkeSpinCorrection(false);
    SmartDashboard.putBoolean("ClarkeSpinCorrectionOn", false);
  }


  public void toggleClarkeSpinCorrectionOnOff(){
    setClarkeSpinCorrection(!isClarkeSpinCorrectionOn);
    SmartDashboard.putBoolean("ClarkeSpinCorrectionOn", isClarkeSpinCorrectionOn);
  }  

  public double getClarkeSpinCorrection(){
    double spinCorrection = 0; 
    SmartDashboard.putString("Clarke.target_color",this.target_color);
    SmartDashboard.putString("Clarke.returned_color",this.returned_color);
    //if (this.returned_color == this.target_color) {
      updateMLData();
      findMinAndMax();
      spinCorrection = peeEyeDee.calculate(center, 0);
    //} 
    return (1 * spinCorrection);
  }


  /**
   * Can be called to force update of VisionClient data structure
   */
  public void updateVisionData(){
    /*tv = piTable.getEntry("tv");
    tx = piTable.getEntry("tx");
    ty = piTable.getEntry("ty");
    tshort = piTable.getEntry("tshort");
    tlong = piTable.getEntry("tlong");
    thor = piTable.getEntry("thor");
    tvert = piTable.getEntry("tvert");
    tcornx = piTable.getEntry("tcornx");
    tcorny = piTable.getEntry("tcorny");
    tl = piTable.getEntry("tl");
    pipeline = piTable.getEntry("pipeline");
    camMode = piTable.getEntry("camMode");
    ledMode = piTable.getEntry("ledMode");
    activePipeline = pipeline.getDouble(0);*/
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run during simulation
    if (idxCounter++ > 1) {
      // A value of 100 in the above conditionals means execution block of conditional will execute every ~2seconds.
 
      updateMLData();
      findMinAndMax();
      //System.out.println(this.maxX);
      //System.out.println(this.minX);
      this.idxCounter = 0;
    } 
    //updateMLData();
  }
  public static Clarke getInstance() {
    return instance;
  }

  public void targetRecogControlLoop(){
    // used to calculate latency
    startTime = Timer.getFPGATimestamp();

    if(tcornx.getDoubleArray(new double[1]).length != 4){
      return;
    }

    deltaXCam = calculateDeltaX();

    // get the initial velocity and angle of ball
    findInitialAngleAndVelocity(0);

    publishAllData();

    // SmartDashboard.putNumber("Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
  }

  private double calculateDeltaX(){
    double[] array = tcornx.getDoubleArray(new double[1]);
    Arrays.sort(array);
    return array[array.length - 1] - array[0];
  }

  public void findInitialAngleAndVelocity(int angleIdx){

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
    // SmartDashboard.putNumber("initialAngle", initialAngle);

    // SmartDashboard.putBoolean("Has Targets", (tv.getDouble(0) == 1));
    // SmartDashboard.putNumber("tshort", tshort.getDouble(0));
    // SmartDashboard.putNumber("tvert", tvert.getDouble(0));

    double numCorners = tcornx.getDoubleArray(new double[0]).length * 2;
    // SmartDashboard.putNumber("Number of Corners", (numCorners > 1) ? numCorners : null);

    // SmartDashboard.putNumber("Radius", radius);
    // SmartDashboard.putNumber("Horizontal Distance", deltaX);
    // SmartDashboard.putNumber("Vertical Distance", deltaY);

    // SmartDashboard.putNumber("Distance According to Camera", deltaXCam);

    // SmartDashboard.putNumber("Approx. Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
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
}
