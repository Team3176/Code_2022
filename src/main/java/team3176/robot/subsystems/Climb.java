// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.littletonrobotics.junction.Logger;
import team3176.robot.subsystems.ClimbIO.ClimbIOInputs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ClimbConstants;
import team3176.robot.constants.MasterConstants;

public class Climb extends SubsystemBase {
  private static Climb instance;
  public static Climb getInstance() {
    if(instance == null) {instance = new Climb(new ClimbIO() {});}
    return instance;
  }

  private final ClimbIO io;
  private final ClimbIOInputs inputs = new ClimbIOInputs();


  private TalonFX winchMotor;
  //TalonFX winchSecondaryMotor;
  private DoubleSolenoid passiveOne;
  private DoubleSolenoid passiveTwo;
  private DoubleSolenoid activeSecondaryOne;
  private DoubleSolenoid activeSecondaryTwo;
  private DigitalInput armOneLimitOne;
  private DigitalInput armOneLimitTwo;
  private DigitalInput armOneLimitThree;
  private DigitalInput armOneLimitFour;
  private DigitalInput armTwoLimitOne;
  private DigitalInput armTwoLimitTwo;
  private DigitalInput armTwoLimitThree;
  private DigitalInput armTwoLimitFour;
  private String armOneState;
  private String armTwoState;
  private boolean isPassivePistonEngaged;
  private boolean isSecondaryPistonEngaged;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = ""; //auto, teleop, test

  private Climb(ClimbIO io) {
    this.io = io;

    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor = new TalonFX(ClimbConstants.FALCON_CAN_ID);
      //winchSecondaryMotor = new TalonFX(ClimbConstants.FALCON2_CAN_ID /*Available Number*/); //TODO:CHECK IF WE NEED MORE NUMBERS
      activeSecondaryOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.ACTIVE_SECONDARY_PISTON_ONE_OPEN_ID, ClimbConstants.ACTIVE_SECONDARY_PISTON_ONE_CLOSE_ID);
      activeSecondaryTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.ACTIVE_SECONDARY_PISTON_TWO_OPEN_ID, ClimbConstants.ACTIVE_SECONDARY_PISTON_TWO_CLOSE_ID);
      armOneLimitTwo = new DigitalInput(ClimbConstants.DIO_ARM_ONE_LIMIT_TWO);
      armOneLimitThree = new DigitalInput(ClimbConstants.DIO_ARM_ONE_LIMIT_THREE);
      armOneLimitFour = new DigitalInput(ClimbConstants.DIO_ARM_ONE_LIMIT_FOUR);
      armTwoLimitTwo = new DigitalInput(ClimbConstants.DIO_ARM_TWO_LIMIT_TWO);
      armTwoLimitThree = new DigitalInput(ClimbConstants.DIO_ARM_TWO_LIMIT_THREE);
      armTwoLimitFour = new DigitalInput(ClimbConstants.DIO_ARM_TWO_LIMIT_FOUR);

      armOneState = new String();
      armTwoState = new String();

      winchMotor.configFactoryDefault();
      // winchSecondaryMotor.configFactoryDefault();

      winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ClimbConstants.SLOTIDX, ClimbConstants.TIMEOUT_MS);
      winchMotor.config_kP(ClimbConstants.SLOTIDX, ClimbConstants.PID_MAIN[0], ClimbConstants.TIMEOUT_MS);
      winchMotor.config_kI(ClimbConstants.SLOTIDX, ClimbConstants.PID_MAIN[1], ClimbConstants.TIMEOUT_MS);
      winchMotor.config_kD(ClimbConstants.SLOTIDX, ClimbConstants.PID_MAIN[2], ClimbConstants.TIMEOUT_MS);
      /*
      winchSecondaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ClimbConstants.SLOTIDX, ClimbConstants.TIMEOUT_MS);
      winchSecondaryMotor.config_kP(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[0], ClimbConstants.TIMEOUT_MS);
      winchSecondaryMotor.config_kI(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[1], ClimbConstants.TIMEOUT_MS);
      winchSecondaryMotor.config_kD(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[2], ClimbConstants.TIMEOUT_MS);
      */
      winchMotor.configPeakOutputForward(.05, ClimbConstants.TIMEOUT_MS); //TODO: TUNNNNNNNEEEEEEEEEEEEEEEEEEEEEEEE
      winchMotor.configPeakOutputReverse(-.05, ClimbConstants.TIMEOUT_MS); //TODO: TUNNNNNNNEEEEEEEEEEEEEEEEEEEEEEEE
      /*
      winchSecondaryMotor.configPeakOutputForward(.05, ClimbConstants.TIMEOUT_MS);
      winchSecondaryMotor.configPeakOutputReverse(-.05, ClimbConstants.TIMEOUT_MS);
      */
      
      isSecondaryPistonEngaged = false;
      isSmartDashboardTestControlsShown = false;
    }
    
    passiveOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.PASSIVE_PISTON_ONE_OPEN_ID, ClimbConstants.PASSIVE_PISTON_ONE_CLOSE_ID);
    passiveTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.PASSIVE_PISTON_TWO_OPEN_ID, ClimbConstants.PASSIVE_PISTON_TWO_CLOSE_ID);

    armOneLimitOne = new DigitalInput(ClimbConstants.DIO_ARM_ONE_LIMIT_ONE);
    armTwoLimitOne = new DigitalInput(ClimbConstants.DIO_ARM_TWO_LIMIT_ONE);

    isPassivePistonEngaged = false;    
  }

  /**
   * Adds Ten to the Position of the Winch Motor
   */

  public void winchManualUp() { //TODO: FIX WITH A RATE THAT I FIND
    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition() + 10);
      // winchSecondaryMotor.set(ControlMode.Position, winchSecondaryMotor.getSelectedSensorPosition() + 10); //TODO: CHECK DIRECTIONS
    }
  }

  /**
   * Subtracts Ten from the Position of the Winch Motor
   */

  public void winchManualDown() { //TODO: FIX WITH A RATE THAT I FIND
    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition() - 10);
      // winchSecondaryMotor.set(ControlMode.Position, winchSecondaryMotor.getSelectedSensorPosition() - 10); //TODO: CHECK DIRECTIONS
    }
  }

  /**
   * Winch Up to the Max Position with 5% of the max speed
   */

  public void winchUp() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor.set(ControlMode.Position, ClimbConstants.WINCH_MAX_LENGTH_POS); //TODO:CHANGE CONSTANT
      // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MAX_LENGTH_POS); //TODO: Add displacement from starting tic
    }
  }

  /**
   * Winch Down to the Min Position with 5% of the max speed
   */

  public void winchDown() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor.set(ControlMode.Position, ClimbConstants.WINCH_MIN_LENGTH_POS); //TODO:CHANGE CONSTANT
      // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MIN_LENGTH_POS); //TODO: Add displacement from starting tic
    }
  }

  /**
   * Extend the Passive Pistons
   */

  public void passivePistonsEngage() {
    passiveOne.set(Value.kForward);
    passiveTwo.set(Value.kForward);
    isPassivePistonEngaged = true;
  }

  /**
   * Retract the Passive Pistons
   */

  public void passivePistonsRetract() {
    passiveOne.set(Value.kReverse);
    passiveTwo.set(Value.kReverse);
    isPassivePistonEngaged = false;
  }

  /**
   * Extend the Secondary Pistons of Both Arm
   */

  public void secondaryPistonsEngage() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      activeSecondaryOne.set(Value.kForward);
      activeSecondaryTwo.set(Value.kForward);
      isSecondaryPistonEngaged = true;
    }
  }

  /**
   * Retract the Secondary Pistons of Both Arm
   */

  public void secondaryPistonsRetract() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      activeSecondaryOne.set(Value.kReverse);
      activeSecondaryTwo.set(Value.kReverse);
      isSecondaryPistonEngaged = false;
    }
  }

  /**
   * Gets the State of Arm One by evaluating the state of all four limit switches
   */

  public void getStageOfArmOne() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      armOneState = "";
      if(getArmOneLimitOne()) {armOneState.concat("1");} else {armOneState.concat("0");}
      if(getArmOneLimitTwo()) {armOneState.concat("1");} else {armOneState.concat("0");}
      if(getArmOneLimitThree()) {armOneState.concat("1");} else {armOneState.concat("0");}
      if(getArmOneLimitFour()) {armOneState.concat("1");} else {armOneState.concat("0");}
    }
  }

  /**
   * Gets the State of Arm Two by evaluating the state of all four limit switches
   */

  public void getStageOfArmTwo() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      armTwoState = "";
      if(getArmTwoLimitOne()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
      if(getArmTwoLimitTwo()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
      if(getArmTwoLimitThree()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
      if(getArmTwoLimitFour()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    }
  }

  /**Returns the State of Arm One gathered by getStateOfArmOne()*/
  public String getArmOneState() {return armOneState;}
  /**Returns the State of Arm Two gathered by getStateOfArmOne()*/
  public String getArmTwoState() {return armTwoState;}
  /**Returns the State of the Primary Pistons*/
  public boolean getPassivePistonEngaged() {return isPassivePistonEngaged;}
  /**Returns the State of the Secondary Pistons*/
  public boolean getSecondaryPistonEngaged() {return isSecondaryPistonEngaged;}

  public boolean getArmOneLimitOne() {return armOneLimitOne.get();}
  public boolean getArmOneLimitTwo() {if(!MasterConstants.ISCLIMBPASSIVE){return armOneLimitTwo.get();}return false;}
  public boolean getArmOneLimitThree() {if(!MasterConstants.ISCLIMBPASSIVE){return armOneLimitThree.get();}return false;}
  public boolean getArmOneLimitFour() {if(!MasterConstants.ISCLIMBPASSIVE){return armOneLimitFour.get();}return false;}
  public boolean getArmTwoLimitOne() {return armTwoLimitOne.get();}
  public boolean getArmTwoLimitTwo() {if(!MasterConstants.ISCLIMBPASSIVE){return armTwoLimitTwo.get();}return false;}
  public boolean getArmTwoLimitThree() {if(!MasterConstants.ISCLIMBPASSIVE){return armTwoLimitThree.get();}return false;}
  public boolean getArmTwoLimitFour() {if(!MasterConstants.ISCLIMBPASSIVE){return armTwoLimitFour.get();}return false;}

  /**
   * Puts Tiles on Shuffleboard (by SmartDashboard) to use in TestPeriodic
   */

  public void putSmartDashboardControlCommands() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      SmartDashboard.putNumber("Winch Falcon PCT", 0);
      // SmartDashboard.putNumber("Secondary Winch Falcon PCT", 0);
      isSmartDashboardTestControlsShown = true;
    }
  }

  /**
   * In TestPeriodic constantly read the values in Shuffleboard
   */

  public void setValuesFromSmartDashboard() {
    if(!MasterConstants.ISCLIMBPASSIVE) {
      winchMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Winch Falcon PCT", 0));
      // wincSecondaryMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Secondary Winch Falcon PCT", 0));
    }
  }

  @Override
  public void periodic() {
    if(mode.equals("test")) {
      if(!MasterConstants.ISCLIMBPASSIVE) {
        if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
        setValuesFromSmartDashboard();
      }
    }

    if(MasterConstants.ISCLIMBPASSIVE && !getArmOneLimitOne() && !getArmTwoLimitOne()) {
      passivePistonsEngage();
    }

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Climb", inputs);
    Logger.getInstance().recordOutput("LimitSwitchOne", getArmOneLimitOne());
    Logger.getInstance().recordOutput("LimitSwitchTwo", getArmTwoLimitOne());
  }

  @Override
  public void simulationPeriodic() {}
}