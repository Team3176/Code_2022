package team3176.robot;

import java.util.Arrays;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3176.robot.constants.*;

/**
 * The VisionClient class is used as a proxy between direct Limelight values and the rest of the Java code.
 * The class also calculates some specific values when used in the context of the 2020/2021 season, such as:
 * -distance from target
 * -horizontal distance from target
 * -vertical distance from target
 * -initial velocity of a ball in order to shoot it into the target
 * -initial angle of the ball in order to shoot it into the target
 */
public class VisionClient{
    // variables to access values given from the Limelight interface
    public NetworkTableInstance tableInstance;
    public NetworkTable limelightTable;
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
    
    private double activePipeline = 1;

    private double deltaXCam;
    private double radius;

    // initializing variables for kinematic calculations
    private final double gravity = -9.81; // m/s^2
    private double deltaX; // m
    private double deltaY; // m
    private double[] initialVelocity = {4.0, 3.0, 2.0}; // m/s
    private double initialTheta; // radians
    private double finalTheta; // radians
    private double xVelocity; // m/s
    private double initialYVelocity; // m/s
    private double finalYVelocity; // m/s
    private double time; // seconds

    //private int ballLocation = -999; // -999=no ball detected, 0=ball to left, 1=ball exactly 0 degrees forward, 2=ball to right
    //private double ballDegrees = -999; // degrees away from Limelight where ball is located. Positive = to left. Negative = to right. Zero = straight ahead.

    /**
     * Creates the default references for VisionClient, specifically for Limelight values
     */
    /*public VisionClient(){
        tableInstance = NetworkTableInstance.getDefault();
        limelightTable = tableInstance.getTable("limelight");
        updateVisionData();

        limelightTable.getEntry("pipeline").setNumber(activePipeline);
    }*/

    /**
     * Can be called to force update of VisionClient data structure
     */
    /*public void updateVisionData(){
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        tshort = limelightTable.getEntry("tshort");
        tlong = limelightTable.getEntry("tlong");
        thor = limelightTable.getEntry("thor");
        tvert = limelightTable.getEntry("tvert");
        tcornx = limelightTable.getEntry("tcornx");
        tcorny = limelightTable.getEntry("tcorny");
        tl = limelightTable.getEntry("tl");
        pipeline = limelightTable.getEntry("pipeline");
        camMode = limelightTable.getEntry("camMode");
        ledMode = limelightTable.getEntry("ledMode");
        activePipeline = pipeline.getDouble(0);
    }*/
    
    /**
     * All calculations for everything are done in this method.
     * It is essentially the main method in this class.
     * It calculates the following values:
     * -distance of camera from target
     * -horizontal distance of camera from target
     * -vertical distance of camera from target
     * -initial angle of ball to make it into target
     * -initial velocity of ball to make it into target
     */
    /*public void targetRecogControlLoop(){
        // used to calculate latency
        double startTime = Timer.getFPGATimestamp();

        publishPrelimTargetRecogData();

        if(tcornx.getDoubleArray(new double[1]).length != 4){
            return;
        }

        calcTargetRecogDistances();

        publishTargetRecogDistances();

        // get the initial velocity and angle of ball
        findInitialAngleAndVelocity(0);

        SmartDashboard.putNumber("Latency (ms)", ((Timer.getFPGATimestamp() - startTime) * 1000) + tl.getDouble(0) + 11);
    }*/


    /**
     * Publishes "Has Targets", tshort, tvert, and tcornxy (under "Length" keyname value) to SmartDashboard
     */
    /*public void publishPrelimTargetRecogData(){
        SmartDashboard.putBoolean("Has Targets", (tv.getDouble(2) == 1));
        SmartDashboard.putNumber("tshort", tshort.getDouble(0));
        SmartDashboard.putNumber("tvert", tvert.getDouble(0));
        double numCorners = tcornx.getDoubleArray(new double[1]).length * 2;
        SmartDashboard.putNumber("Number of Corners", (numCorners > 1) ? numCorners : null);
    }*/

    /**
     * Calculates distances (deltaX and deltaY) to target from Target Recog Data
     */ 
    /*public void calcTargetRecogDistances(){
        
        // calculate the distance between the furthest two points as the camera sees it
        deltaXCam = calculateDeltaX(tcornx);

        SmartDashboard.putNumber("DeltaXCam", deltaXCam);

        // calculate the various kinds of distances from the camera
        radius = VisionConstants.VISION_CONSTANT / deltaXCam;
        deltaX = radius * Math.cos(ty.getDouble(0) * VisionConstants.DEG2RAD);
        deltaY = radius * Math.sin(ty.getDouble(0) * VisionConstants.DEG2RAD);
    }*/


    /*public void calcTargetRecogDistances(){
        deltaXCam = calculateDeltaX();

        SmartDashboard.putNumber("DeltaXCam", deltaXCam);
    }*/

    /**
     * Publishes Target Recog Data to SmartDashboard.  Variables published are: radius, deltaX, deltaY.
     */
    /*public void publishTargetRecogDistances(){
        SmartDashboard.putNumber("Radius", radius);
        SmartDashboard.putNumber("deltaX", deltaX);
        SmartDashboard.putNumber("deltaY", deltaY);
    }*/
    
    /**
     * Calculates the difference in the two points furthest away from each other
     * 
     * @param array array to find the range of
     * @return double that represents the range of the array
     */
    /*public double calculateDeltaX(){
        double[] array = tcornx.getDoubleArray(new double[1]);
        Arrays.sort(array);
        return array[array.length - 1] - array[0];
    }*/


    /**
     * Calculates the initial angle and velocity of the ball using kinematic equations
     * 
     * @param speedIdx The index in the possible speed array (above) for the method to start at. It is reccommended to default to 0.
     * @return A double array where the first value is the velocity, and the second value is the angle.
     */
    public double[] findInitialAngleAndVelocity(int speedIdx){
        if(speedIdx >= initialVelocity.length){
            speedIdx = 0;
        }

        findInitialAngle(initialVelocity[speedIdx]);
        //publishPrelimTargetRecogData();

        if(!(initialTheta >= 0 && initialTheta <= (29.2 * VisionConstants.DEG2RAD))){
            if(speedIdx < initialVelocity.length - 1){
                return findInitialAngleAndVelocity(speedIdx + 1);
            } else{
                return null;
            }
        }

        solveOtherVariablesFromAngle();

        // figures out if the solution is valid by checking if it would actually go into the target
        if(finalTheta >= (11 * Math.PI)/12 && finalTheta <= (13 * Math.PI)/12){
            double[] arrayToSend = {initialVelocity[speedIdx], initialTheta};
            return arrayToSend;
        } else{
            if(speedIdx < initialVelocity.length - 1){
                return findInitialAngleAndVelocity(speedIdx + 1);
            } else {
                return null;
            }
        }
    }


    /**
     * Calculates the initial angle of the ball using kinematic equations
     * 
     * @param speedIdx The index in the possible speed array (above) for the method to start at. It is recommended to default to 0.
     */
    public void findInitialAngle(double speed){
        // the single line that calculates the initial angle
        initialTheta = Math.atan((-deltaX + Math.sqrt(Math.pow(deltaX, 2) - 4 * ((gravity * deltaX) / (2 * Math.pow(speed, 2))) * (((gravity * deltaX) / (2 * Math.pow(speed, 2))) - deltaY))) / ((gravity * deltaX) / Math.pow(speed, 2)));
    }


    /** 
     * Publishes initialAngle to SmartDashboard under key value "initialTheta".
     */
    /*public void publishInitialTheta(){
        SmartDashboard.putNumber("initialTheta", initialTheta);
    }*/


    /**
     * Calculates the other variables needed to verify the solution
     */
    public void solveOtherVariablesFromAngle(){
        // this section calculates the angle that the ball would approach the target to see if it would actually go in
        xVelocity = Math.cos(initialTheta);
        initialYVelocity = Math.sin(initialTheta);

        time = ((-initialYVelocity - Math.sqrt(Math.pow(initialYVelocity, 2) - 4 * (.5 * gravity) * (-1 * deltaY))) / (gravity));

        finalYVelocity = initialYVelocity + (gravity * time);

        finalTheta = Math.atan(finalYVelocity / xVelocity) + Math.PI;
    }


    /**
     * Determines if a ball is detected, and if so how many degrees to left or right of Limelight crosshairs.
     * Negative degrees = Ball to Right of crosshairs.
     * Positive degrees = Ball to Left of crosshairs.
     * Zero degrees = Ball straight ahead forward.
     */
    /*public void controlLoopBallRecog(){
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        limelightTable.getEntry("pipeline").setNumber(0);
        
        double hasTarget = tv.getDouble(0);
        double targetX = tx.getDouble(0);
        double absOffset = Math.abs(targetX);
        if(hasTarget == 1){
          SmartDashboard.putBoolean("Ball Recognized", true);
          if(targetX <= 1 && targetX >= -1){
            ballLocation = 1;
            ballDegrees = 0;
            SmartDashboard.putNumber("Degrees", 0);
          } else if(targetX >= 1){
            ballLocation = 2;
            ballDegrees = absOffset * -1;
              //Ball on the Right side of Limelight crosshairs by absOffset Degrees.
            SmartDashboard.putNumber("Ball Degrees", absOffset * -1);
          } else if(targetX <= -1){
            ballLocation = 0;
            ballDegrees = absOffset;
              // Ball on the Left side of Limelight crosshairs by absOffset Degrees.
            SmartDashboard.putNumber("Ball Degrees", absOffset);
          } else{
            ballDegrees = -999;
            System.out.println("Ball Recog FAILED:  see VisionClient.controlLoopBallRecog.");
          }
        } else if (hasTarget == 0){
            ballLocation = -999;
            SmartDashboard.putBoolean("Ball Recognized", false);
        } else{
          System.out.println("Ball Recog FAILED:  see VisionClient.controlLoopBallRecog.");
        }
    }*/



    /**
     * Turns on Limelight's LEDs.  Duh.
     */
    /*public void turnLEDsOn(){
        ledMode.setNumber(3);
    }*/


    /**
     * Blinks Limelight's LEDs.  Double duh.
     */
    /*public void blinkLEDs(){
        ledMode.setNumber(2);
    }*/


    /**
     * Seriously, man? Method's name says it all. It turns off Limelight's LEDs
     */
    /*public void turnLEDsOff(){
        ledMode.setNumber(1);
    }*/


    /**
     * Sets camera's mode
     * @param mode determines which mode the method is set to, true is vision processing, false is Driver Cam.
     */
    public void setCameraMode(boolean mode){
        if(mode){
            camMode.setNumber(0);
        } else{
            camMode.setNumber(1);
        }
    }

    public boolean getCameraMode(){
        return camMode.getNumber(0).equals(0.0);
    }


   /**
   * Gets which pipeline the processor will use. Returns double value indicating number of currently active pipeline. 
   */
    public Double getActivePipeline(){
        activePipeline = pipeline.getDouble(0);
        return activePipeline;
    }


   /**
   * Sets which pipeline the processor will use.
   * @param desiredPipelineNum sets the pipeline that will be used. Acceptable values are 0 and 1 at present.
   */
    public void setActivePipeline(double desiredPipelineNum){
        if((int) desiredPipelineNum >= 0 && (int) desiredPipelineNum <= 5){
            activePipeline = desiredPipelineNum;
            pipeline.setNumber(activePipeline);
        } else{
            System.out.println("Invalid Pipeline Requested; No Change Was Made");
        }
    }

    /*public double getBallLocation(){
        return ballLocation;
    }

    public double getBallDegrees(){
        return ballDegrees;
    }*/
}
