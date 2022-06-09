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

import team3176.robot.util.God.PID3176;
import team3176.robot.constants.DrivetrainConstants;
// import team3176.robot.util.God.PID3176;
import team3176.robot.subsystems.drivetrain.SwervePod2022;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.subsystems.Controller;
import team3176.robot.subsystems.Vision;
import team3176.robot.subsystems.Clarke;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.Gyro3176;

import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.drivetrain.DrivetrainIO.DrivetrainIOInputs;



  

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;
  private CoordSys m_CoordSys = CoordSys.getInstance();
  private Gyro3176 m_Gyro3176 = Gyro3176.getInstance();
  private Vision m_Vision = Vision.getInstance();
  private Clarke m_Clarke = Clarke.getInstance();

  //private Controller controller = Controller.getInstance();
  //private Vision m_Vision = Vision.getInstance();

 // private PowerDistribution PDP = new PowerDistribution(PowerManagementConstants.PDP_CAN_ID, ModuleType.kCTRE);

  private ArrayList<SwervePod2022> pods;

  private driveMode currentDriveMode;

  public TalonFX[] driveControllers = { new TalonFX(DrivetrainConstants.THRUST_ONE_CID),
      new TalonFX(DrivetrainConstants.THRUST_TWO_CID), new TalonFX(DrivetrainConstants.THRUST_THREE_CID),
      new TalonFX(DrivetrainConstants.THRUST_FOUR_CID) };

  public CANSparkMax[] azimuthControllers = { new CANSparkMax(DrivetrainConstants.STEER_ONE_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_TWO_CID, MotorType.kBrushless), new CANSparkMax(DrivetrainConstants.STEER_THREE_CID, MotorType.kBrushless),
      new CANSparkMax(DrivetrainConstants.STEER_FOUR_CID, MotorType.kBrushless) };

  private double length; // robot's wheelbase
  private double width; // robot's trackwidth

  private double relMaxSpeed;

  private double forwardCommand;
  private double strafeCommand;
  private double spinCommand;
  private double spinCommandInit;

  // private PID3176 spinLockPID;
  // private PIDController spinLockPID;

  private boolean isTurboOn = false;

  private int arraytrack;
  double[] angleHist = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  double angleAvgRollingWindow;

  public enum driveMode {
    DEFENSE, DRIVE, VISION, ORBIT, PIVOTFRFL, PIVOTFLBL, PIVOTBLBR, PIVOTBRFR, PIVOTBRBL, PIVOTFRBR, PIVOTFLFR, PIVOTBLFL
  }

  private SwervePod2022 podFR;
  private SwervePod2022 podFL;
  private SwervePod2022 podBL;
  private SwervePod2022 podBR;



  private final DrivetrainIO io;
  //private final DrivetrainIOInputs inputs = new DrivetrainIOInputs();

  private Drivetrain(DrivetrainIO io) {
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


    // Setting constants
    length = DrivetrainConstants.LENGTH;
    width = DrivetrainConstants.WIDTH;
    
    //SmartDashboard.putNumber("currentAngle", m_Gyro3176.getCurrentChassisYaw());

    // SmartDashboard.putNumber("forwardCommand", 0);
    // SmartDashboard.putNumber("strafeCommand", 0);
    // SmartDashboard.putNumber("spinCommand", 0);

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
    this.spinCommandInit = -spinCommand;
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
      //this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
      this.spinCommand *= DrivetrainConstants.NON_TURBO_PERCENT_OUT_CAP;
       //for (int idx = 0; idx < (pods.size()); idx++) {
       //  pods.get(idx).unboostThrustAcceleration();
       //}
    }

    if (isTurboOn) {
      this.spinCommand *= 2; 
       //for (int idx = 0; idx < (pods.size()); idx++) {
       //  pods.get(idx).boostThrustAcceleration();
       //}
    }

    if (m_Gyro3176.getIsSpinLocked() && !isOrbiting()) {
      this.spinCommand = m_Gyro3176.getSpinLockPIDCalc();
    }

    if (m_Vision.getIsVisionSpinCorrectionOn()) {
      this.spinCommand = m_Vision.getVisionSpinCorrection();
    }
    
    if (m_Clarke.getIsClarkeSpinCorrectionOn()) {
      this.spinCommand = m_Clarke.getClarkeSpinCorrection(); 
      SmartDashboard.putNumber("Drivetrain_ClarkeSpinCommand",this.spinCommand);
    }

    if (m_CoordSys.isFieldCentric()) {

      // System.out.println("Drivetrain ran under isFieldCentric -----------------------------------------------------------------------------------------------------------------------------------");

      double currentAngle = m_Gyro3176.getCurrentChassisYaw();
      final double temp = (this.forwardCommand * Math.cos(m_Gyro3176.getCurrentChassisYaw())
          + this.strafeCommand * Math.sin(m_Gyro3176.getCurrentChassisYaw()));
      this.strafeCommand = (-this.forwardCommand * Math.sin(m_Gyro3176.getCurrentChassisYaw())
          + this.strafeCommand * Math.cos(m_Gyro3176.getCurrentChassisYaw()));
      // TEST BELOW TO SEE IF FIXES RC/FC ALIGNMENT
      // final double temp = (this.forwardCommand * Math.sin(m_Gyro3176.getCurrentChassisYaw())
      // + this.strafeCommand * Math.cos(m_Gyro3176.getCurrentChassisYaw()));
      // this.strafeCommand = (-this.forwardCommand * Math.cos(m_Gyro3176.getCurrentChassisYaw())
      // + this.strafeCommand * Math.sin(m_Gyro3176.getCurrentChassisYaw()));
      this.forwardCommand = temp;
      SmartDashboard.putBoolean("isFieldCentricOn", true);
    }
    
    if (m_CoordSys.isRobotCentric()) {
      this.strafeCommand *= 1; // 0.75;
      this.forwardCommand *= 1; // 0.75;
      this.spinCommand *= 1; // 0.75;
      SmartDashboard.putBoolean("isFieldCentricOn", false);
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
    //pods.get(1).tune();
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
//      if (! (currentDriveMode == driveMode.PIVOTFR)) {  
      relMaxSpeed = Math.max(Math.max(podDrive[0], podDrive[1]), Math.max(podDrive[2], podDrive[3]));
      if (relMaxSpeed > DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND) {
        for (int idx = 0; idx < pods.size(); idx++) {
          podDrive[idx] /= relMaxSpeed / DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND;
        }
      }
 //     }

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
      double smallNum = Math.pow(10, -5);
      /*
      // OLD DEFENSE
      pods.get(0).set(smallNum, -1.0 * Math.PI / 4.0);
      pods.get(1).set(smallNum, 1.0 * Math.PI / 4.0);
      pods.get(2).set(smallNum, 3.0 * Math.PI / 4.0);
      pods.get(3).set(smallNum, -3.0 * Math.PI / 4.0);
      */
      // NEW DEFENSE
      pods.get(0).set(smallNum, 1.0 * Math.PI / 4.0);
      pods.get(1).set(smallNum, -1.0 * Math.PI / 4.0);
      pods.get(2).set(smallNum, -3.0 * Math.PI / 4.0);
      pods.get(3).set(smallNum, 3.0 * Math.PI / 4.0);
    }
  }

  public void stopMotors() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      driveControllers[idx].set(ControlMode.PercentOutput, 0);
      azimuthControllers[idx].set(0);
    }

  }

  public void setPodsAzimuthHome() {
      double smallNum = Math.pow(10, -5);
      pods.get(0).set(smallNum, 0 * Math.PI / 4.0);
      pods.get(1).set(smallNum, 0 * Math.PI / 4.0);
      pods.get(2).set(smallNum, 0 * Math.PI / 4.0);
      pods.get(3).set(smallNum, 0 * Math.PI / 4.0);
  }

  public void sendPodsAzimuthToHome() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).goHome();
    }
  }

  public void setCurrentPodPosAsHome() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setCurrentPositionAsHome();
    }
  }

  public void setSwerveKillSwitchOn() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setKillSwitchOn();
    }
  }


  public void setSwerveKillSwitchOff() {
    for (int idx = 0; idx < (pods.size()); idx++) {
      pods.get(idx).setKillSwitchOff();
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
   
    // }

    if (currentDriveMode == driveMode.DRIVE) {
      if (component.equals("A") || component.equals("B")) {
        return length / 2.0;
      } else {
        return width / 2.0;
      } // TODO: place to check for forward vs back pods working vs not working
    }

    String pivotpoint = ""; 
    if (currentDriveMode == driveMode.PIVOTFRFL) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTFR"; }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFL";}
    }
    if (currentDriveMode == driveMode.PIVOTFLBL) {
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFL"; }
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBL";}
    }
    if (currentDriveMode == driveMode.PIVOTBLBR) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBL"; 
         }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTBR"; this.spinCommand = -this.spinCommand;}
    }
    if (currentDriveMode == driveMode.PIVOTBRFR) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBR"; }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFR";}
    }
    if (currentDriveMode == driveMode.PIVOTBRBL) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBR"; }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTBL";}
    }
    if (currentDriveMode == driveMode.PIVOTFRBR) {
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFR"; this.spinCommand = -1 * this.spinCommand; }
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBR"; this.spinCommand = -1 * this.spinCommand;}
    }
    if (currentDriveMode == driveMode.PIVOTFLFR) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTFL"; }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFR";}
    }
    if (currentDriveMode == driveMode.PIVOTBLFL) {
      if (spinCommandInit < 0) { pivotpoint = "PIVOTBL"; }
      if (spinCommandInit > 0) { pivotpoint = "PIVOTFL";}
    }
    switch(pivotpoint) {
      case "PIVOTFR": if (component.equals("B") || component.equals("C")) { 
                        return 0.0;
                      } else if (component.equals("A")) {
                        return length;
                      } else if (component.equals("D")) {
                        return width; 
                      } 
                      break;
      case "PIVOTFL": if (component.equals("B") || component.equals("D")) {
                        return 0.0;
                      } else if (component.equals("A")) {
                        return length;
                      } else if (component.equals("C")) {
                        return width;
                      }
                      break;
      case "PIVOTBL": if (component.equals("A") || component.equals("D")) {
                        return 0.0;
                      } else if (component.equals("B")) {
                        return length;
                      } else if (component.equals("C")) {
                        return width;
                      }
                      break;
      case "PIVOTBR": if (component.equals("A") || component.equals("C")) {
                        return 0.0;
                      } else if (component.equals("B")) { 
                        return length;
                      } else if (component.equals("D")) {
                        return width;
                      }
    }

    /*
    if (currentDriveMode == driveMode.PIVOTFR) {
      if (component.equals("B") || component.equals("C")) {
        return 0.0;
      } else if (component.equals("A")) {
        return length;
      } else if (component.equals("D")) {
        return width; 
      } 
    }

    if (currentDriveMode == driveMode.PIVOTFL) {
      if (component.equals("B") || component.equals("D")) {
        return 0.0;
      } else if (component.equals("A")) {
        return length;
      } else if (component.equals("C")) {
        return width;
      }
    }

    if (currentDriveMode == driveMode.PIVOTBL) {
      if (component.equals("A") || component.equals("D")) {
        return 0.0;
      } else if (component.equals("B")) {
        return length;
      } else if (component.equals("C")) {
        return width;
      }
    }

    if (currentDriveMode == driveMode.PIVOTBR) {
      if (component.equals("A") || component.equals("C")) {
        return 0.0;
      } else if (component.equals("B")) { 
        return length;
      } else if (component.equals("D")) {
        return width;
      }
    }

    */

     return 0.0; // TODO: this method needs cleanup and logic-checking
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

  public double getAngleAvgRollingWindow() {
    return m_Gyro3176.getAngleAvgRollingWindow();
  }


  public double getPodVelocity(int podID) {
    return pods.get(podID).getVelocity();
  }

  public double  getPodAzimuth(int podID) {
    return pods.get(podID).getAzimuth();
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
    
    this.arraytrack++;
    if (this.arraytrack > 3) {
      this.arraytrack = 0;
    }

    SmartDashboard.putBoolean("Turbo", isTurboOn);
    // SmartDashboard.putBoolean("Defense", currentDriveMode == driveMode.DEFENSE);
    SmartDashboard.putBoolean("SpinLock", m_Gyro3176.getIsSpinLocked());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
