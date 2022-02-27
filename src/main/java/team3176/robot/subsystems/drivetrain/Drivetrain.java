// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.drivetrain;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import team3176.robot.constants.DrivetrainConstants;
// import team3176.robot.util.God.PID3176;
import team3176.robot.subsystems.drivetrain.SwervePod2022;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.Controller;
import team3176.robot.subsystems.Vision;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.Gyro3176;

import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;



  

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_Gyro3176 = Gyro3176.getInstance();

  //private Controller controller = Controller.getInstance();
//private Vision m_Vision = Vision.getInstance();

 // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);

  private ArrayList<SwervePod2022> pods;

  private driveMode currentDriveMode;

  private boolean autonVision;

  public TalonFX[] driveControllers = { new TalonFX(DrivetrainConstants.THRUST_ONE_CID),
      new TalonFX(DrivetrainConstants.THRUST_TWO_CID), new TalonFX(DrivetrainConstants.THRUST_THREE_CID),
      new TalonFX(DrivetrainConstants.THRUST_FOUR_CID) };

  public CANSparkMax[] azimuthControllers = { new CANSparkMax(DrivetrainConstants.STEER_ONE_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_TWO_CID, MotorType.kBrushless), new CANSparkMax(DrivetrainConstants.STEER_THREE_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_FOUR_CID, MotorType.kBrushless) };

  private double length; // robot's wheelbase
  private double width; // robot's trackwidth
  private double k_etherRadius; // radius used in A,B,C,D component calc's of ether decomposition

  private double maxSpeed_InchesPerSec;
  private double maxVel;
  private double maxRotation;
  private double maxAccel;

  private double relMaxSpeed;
  private double currentAngle;
  private double lastAngle;

  private double startTime = 0;
  private double currentTIme = 0;

  private boolean isVisionDriving;

  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;

  private double spinLockAngle;
  private boolean isSpinLocked = false;
  // private PID3176 spinLockPID;
  // private PIDController spinLockPID;

  private boolean isTurboOn = false;
  
  private int spinEncoderIdxCount = 0;

  private int arraytrack;
  double[] angleHist = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  double angleAvgRollingWindow;

  public enum driveMode {
    DEFENSE, DRIVE, VISION, ORBIT
  }

  private SwervePod2022 podFR;
  private SwervePod2022 podFL;
  private SwervePod2022 podBL;
  private SwervePod2022 podBR;

  private double lockP, lockI, lockD;

  private final DrivetrainIO io;
  private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(DrivetrainIO io) 
  {
    this.io = io;
    
    // Instantiate pods
    podFR = new SwervePod2022(0, driveControllers[0], azimuthControllers[0]);
    podFL = new SwervePod2022(1, driveControllers[1], azimuthControllers[1]);
    podBL = new SwervePod2022(2, driveControllers[2], azimuthControllers[2]);
    podBR = new SwervePod2022(3, driveControllers[3], azimuthControllers[3]);

    // Instantiate array list then add instantiated pods to list
    pods = new ArrayList<SwervePod2022>();
    pods.add(podFR);
    pods.add(podFL);
    pods.add(podBL);
    pods.add(podBR);

    autonVision = false;

    // Setting constants
    length = DrivetrainConstants.LENGTH;
    width = DrivetrainConstants.WIDTH;
    k_etherRadius = Math.sqrt(Math.pow(length, 2) / Math.pow(width, 2)) / 2;

    maxSpeed_InchesPerSec = DrivetrainConstants.MAX_WHEEL_SPEED_INCHES_PER_SECOND;
    maxRotation = DrivetrainConstants.MAX_ROT_SPEED;
    maxAccel = DrivetrainConstants.MAX_ACCEL;

    // SmartDashboard.putNumber("currentAngle", this.currentAngle);

    // SmartDashboard.putNumber("forwardCommand", 0);
    // SmartDashboard.putNumber("strafeCommand", 0);
    // SmartDashboard.putNumber("spinCommand", 0);

    isVisionDriving = false;

    arraytrack = 0;
    angleAvgRollingWindow = 0;

    // TODO: We initialize to face forward but how do we make this into a command?
    // Maybe we say drive with the below parameters, but where?
    /*
     * // Start wheels in a forward facing direction
     */

    this.forwardCommand = Math.pow(10, -15); // Has to be positive to turn that direction?
    this.strafeCommand = 0.0;
    this.spinCommand = 0.0;

 
  }

  // Prevents more than one instance of drivetrian
  public static Drivetrain getInstance() {
    if(instance == null) {instance = new Drivetrain(new DrivetrainIO() {});}
    return instance;
  }

  /**
   * public void drive(double forwardCommand, double strafeCommand, double
   * spinCommand, int uselessVariable) { double smallNum = Math.pow(10, -15);
   * //spinCommand = (spinCommand - (-1))/(1 - (-1)); //rescales spinCommand to a
   * 0..1 range double angle = (spinCommand * Math.PI) + Math.PI; // <- diff coord
   * system than -1..1 = 0..2Pi // This coord system is 0..1 = Pi..2Pi, & // 0..-1
   * = Pi..-2PI // right? // Fixed by new rescaling at line 140?
   * pods.get(0).set(smallNum, angle); }
   */

  /**
   * 
   * @param forwardCommand feet per second
   * @param strafeCommand  feet per second
   * @param spinCommand    feet per second
   */
  public void drive(double forwardCommand, double strafeCommand, double spinCommand) {
    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;  // TODO: The y is inverted because it is backwards for some reason, why?
    this.spinCommand = spinCommand;
    // System.out.println("Forward Command" + forwardCommand);

    // this.forwardCommand = SmartDashboard.getNumber("forwardCommand", 0);
    // this.strafeCommand = SmartDashboard.getNumber("strafeCommand", 0);
    // this.spinCommand = SmartDashboard.getNumber("spinCommand", 0);

    // SmartDashboard.putNumber("drive()InputForwardCommand", forwardCommand);
    // SmartDashboard.putNumber("drive()InputStrafeCommand", strafeCommand);
    // SmartDashboard.putNumber("drive()InputSpinCommand", spinCommand);

 
    if (!isTurboOn) {
      this.forwardCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.strafeCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
    }
    if (this.isSpinLocked && !isOrbiting()) {
      // this.spinCommand = -spinLockPID.returnOutput(m_Gyro3176.getNavxAngle_inRadians(), spinLockAngle);
      // this.spinCommand = spinLockPID.calculate(getNavxAngle(), spinLockAngle);

    }

    if (m_CoordSys.isFieldCentric()) {
      final double temp = (this.forwardCommand * Math.cos(this.currentAngle)
          + this.strafeCommand * Math.sin(this.currentAngle));
      this.strafeCommand = (-this.forwardCommand * Math.sin(this.currentAngle)
          + this.strafeCommand * Math.cos(this.currentAngle));
      // TEST BELOW TO SEE IF FIXES RC/FC ALIGNMENT
      // final double temp = (this.forwardCommand * Math.sin(this.currentAngle)
      // + this.strafeCommand * Math.cos(this.currentAngle));
      // this.strafeCommand = (-this.forwardCommand * Math.cos(this.currentAngle)
      // + this.strafeCommand * Math.sin(this.currentAngle));
      this.forwardCommand = temp;
    }
    // TODO: Find out why we multiply by 0.75
    if (m_CoordSys.isRobotCentric()) {
      this.strafeCommand *= 1; // 0.75;
      this.forwardCommand *= 1; // 0.75;
      this.spinCommand *= 1; // 0.75;
    }

    // SmartDashboard.putNumber("this.forwardCom_Drivetrain.drive",
    // this.forwardCommand);
    // SmartDashboard.putNumber("this.strafeCom_Drivetrain.drive",
    // this.strafeCommand);
    // TODO: Find out why this putNumber statement is making the spinLock work
    // SmartDashboard.putNumber("this.spinCom_Drivetrain.drive", this.spinCommand);
    calculateNSetPodPositions(this.forwardCommand, this.strafeCommand, this.spinCommand);
    //for (int idx = 0; idx < (pods.size()); idx++) {
    //  pods.get(idx).tune();
    //}
    //pods.get(0).tune();
  }

  /**
   * 
   * @param forwardCommand feet per second
   * @param strafeCommand  feet per second
   * @param spinCommand    feet per second
   */
  private void calculateNSetPodPositions(double forwardCommand, double strafeCommand, double spinCommand) {

    this.forwardCommand = forwardCommand;
    this.strafeCommand = strafeCommand;
    this.spinCommand = spinCommand;

    if (currentDriveMode != driveMode.DEFENSE) {
      // Create arrays for the speed and angle of each pod
      double[] podDrive = new double[4];
      double[] podSpin = new double[4];

      // ###########################################################
      // BEGIN: Ether Eqns -- Ether's official derivation
      // +Y := axis of chassis forward movement
      // +X := axis of chassis strafe to starboard/right
      // ###########################################################
      double a = strafeCommand - spinCommand * getRadius("A");
      double b = strafeCommand + spinCommand * getRadius("B");
      
      double c = forwardCommand - spinCommand * getRadius("C");
      double d = forwardCommand + spinCommand * getRadius("D");

      // Calculate speed (podDrive[idx]) and angle (podSpin[idx]) of each pod
      podDrive[0] = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
      podSpin[0] = Math.atan2(b, c);

      podDrive[1] = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
      podSpin[1] = Math.atan2(b, d);

      podDrive[2] = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
      podSpin[2] = Math.atan2(a, d);

      podDrive[3] = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
      podSpin[3] = Math.atan2(a, c);
      // ###########################################################
      // /END Ether Eqns -- Ether's official derivation
      // ###########################################################

      /*
       * // ########################################################### // BEGIN:
       * Ether Eqns -- JonH derivation 2021-02-15 // +X := axis of chassis forward
       * movement ... we think // -Y := axis of chassis strafe to starboard/right ...
       * we think // ###########################################################
       * double a = this.forwardCommand - this.spinCommand * width/2; double b =
       * this.forwardCommand + this.spinCommand * width/2; double c =
       * this.strafeCommand - this.spinCommand * length/2; double d =
       * this.strafeCommand + this.spinCommand * length/2;
       * 
       * // Calculate speed and angle of each pod // TODO: Verify order of atan2
       * parameters. atan2(y,x) is formal java def, // but past implementations and
       * ether use atan2(x,y). podDrive[0] = Math.sqrt(Math.pow(b,2) + Math.pow(d,2));
       * podSpin[0] = Math.atan2(d, b);
       * 
       * podDrive[1] = Math.sqrt(Math.pow(b,2) + Math.pow(c,2)); podSpin[1] =
       * Math.atan2(c, b);
       * 
       * podDrive[2] = Math.sqrt(Math.pow(a,2) + Math.pow(c,2)); podSpin[2] =
       * Math.atan2(c, a);
       * 
       * podDrive[3] = Math.sqrt(Math.pow(a,2) + Math.pow(d,2)); podSpin[3] =
       * Math.atan2(d, a); //
       * ########################################################### // END: Ether
       * Eqns -- JonH derivation 2021-02-15 //
       * ###########################################################
       */

      // SmartDashboard.putNumber("a", a);
      // SmartDashboard.putNumber("b", b);
      // SmartDashboard.putNumber("c", c);
      // SmartDashboard.putNumber("d", d);
      for (int idx = 0; idx < 4; idx++) {
        // SmartDashboard.putNumber("preScale P" + (idx + 1) + " podDrive",
        // podDrive[idx]);
      }

      // Find the highest pod speed then normalize if a pod is exceeding our max speed by scaling down all the speeds
      relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
      if (relMaxSpeed > DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND) {
        for (int idx = 0; idx < pods.size(); idx++) {
          podDrive[idx] /= relMaxSpeed / DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND;
        }
      }

      // Set calculated drive and spins to each pod
      // for(int idx = 0; idx < pods.size(); idx++) {
      for (int idx = 0; idx < (pods.size()); idx++) {
        pods.get(idx).set(podDrive[idx], podSpin[idx]); // TODO: try doing pods.size() - 1 in for conditional, then
                                                        // outside for loop
                                                        // do a hardcode set of pods.get(3).set(0.1, 0.0);
        // SmartDashboard.putNumber("pod" + idx + " drive", podDrive[idx]);
        // SmartDashboard.putNumber("pod" + idx + " spin", podSpin[idx]);
      }
      // pods.get(3).set(0.1,1.57);


      //SmartDashboard.putBoolean("orbiting", isOrbiting());
    } else if (currentDriveMode == driveMode.DEFENSE) { // Enter defensive position
      double smallNum = Math.pow(10, -15);
      pods.get(0).set(smallNum, -1.0 * Math.PI / 4.0);
      pods.get(1).set(smallNum, 1.0 * Math.PI / 4.0);
      pods.get(2).set(smallNum, 3.0 * Math.PI / 4.0);
      pods.get(3).set(smallNum, -3.0 * Math.PI / 4.0);
    }
  }

  public void stopMotors() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      driveControllers[idx].set(ControlMode.PercentOutput, 0);
      azimuthControllers[idx].set(0);
    }

  }

  private double getRadius(String component) {
    // Omitted if driveStatements where we pivoted around a pod
    // This'll be orbit and dosado in the future
    // if(currentDriveMode == driveMode.ORBIT) {
    // if(component.equals("A") || component.equals("B")) { return length / 2.0; }
    // else if(component.equals("C")) { return width; }
    // else /* component D */ { return 2 * width; } // Puts radius to the right of
    // bot at distance w
    // } else {
    if (component.equals("A") || component.equals("B")) {
      return length / 2.0;
    } else {
      return width / 2.0;
    } // TODO: place to check for forward vs back pods working vs not working
    // }
    /// return 0.0; // TODO: this method needs cleanup and logic-checking
  }

  public void setDriveMode(driveMode wantedDriveMode) {
    currentDriveMode = wantedDriveMode;
  }

  public boolean isOrbiting() {
    if (currentDriveMode == driveMode.ORBIT) {
      return true;
    }
    return false;
  }


  
 
  public driveMode getCurrentDriveMode() {
    return currentDriveMode;
  }

  
  


  public void toggleSpinLock() {
    this.isSpinLocked = !this.isSpinLocked;
  }

  public void setSpinLock(boolean set) {
    isSpinLocked = set;
  }

 
  /**
   * Sets Turbo mode on or off
   * @param onOrOff Passing a value of true sets Turbo on (ie isTurboOn = true), and passing value of false sets Turbo off (ie isTurboOn = false)
   */
  public void setTurbo(boolean onOrOff) {
    this.isTurboOn = onOrOff;
  }

  /*
   * public void drive(double drivePercent, double spinPercent) {
   * SmartDashboard.putBoolean("Are we calling drive", true);
   * pod1.velocityPIDDriveNSpin(drivePercent, spinPercent); //
   * pod1.percentFFDriveNSpin(drivePercent, spinPercent); if(drivePercent > 1.4) {
   * pod.velocityPIDDriveNSpin(5.0, 0.0); } else { pod.velocityPIDDriveNSpin(0.0,
   * 0.0); } }
   */

  /*

   * 
 
   */

   /** 
    * Calculates average angle value based on rolling window of last five angle measurements
    */
  public void calcAngleAvgRollingWindow() {
    this.angleHist[this.arraytrack] = this.currentAngle;
    angleAvgRollingWindow = (this.angleHist[0] + this.angleHist[1] + this.angleHist[2] + this.angleHist[3]
        + this.angleHist[4]) / 5;
  }

  public double getAngleAvgRollingWindow() {
    return this.angleAvgRollingWindow;
  }


  
/*
  public ChassisSpeeds getChassisSpeed() {
    return DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(podFR.getState(), podFL.getState(), podBL.getState(), podBR.getState());
  }
*/


 /* public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(
      desiredStates, 4.468); //Unist.inchesToMeters(DrivetrainConstants.MAX_WHEEL_SPEED_INCHES_PER_SECOND));
    
    podFR.setDesiredState(desiredStates[0]);
    podFL.setDesiredState(desiredStates[1]);
    podBL.setDesiredState(desiredStates[2]);
    podBR.setDesiredState(desiredStates[3]);

  }
  */


  @Override
  public void periodic() {
    // This method will be called once per scheduler every 500ms
    if(spinEncoderIdxCount++ > 25) { 
      for (int idx = 0; idx < (pods.size()); idx++) { 
        spinEncoderIdxCount = 0;
        pods.get(idx).updateAzimuthEncoder(); 
        //pods.get(0).podAzimuth = SmartDashboard.getNumber("P0.podSpin_setpoint_angle",0);
      }
    }
    
    calcAngleAvgRollingWindow();
    this.arraytrack++;
    if (this.arraytrack > 3) {
      this.arraytrack = 0;
    } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
