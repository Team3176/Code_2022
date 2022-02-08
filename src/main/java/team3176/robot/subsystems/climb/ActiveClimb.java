// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ActiveClimbConstants;

public class ActiveClimb extends SubsystemBase {
  private static ActiveClimb instance = new ActiveClimb();
  public static ActiveClimb getInstance() {return instance;}

  private TalonFX winchMotor;
  //TalonFX winchSecondaryMotor;
  private DoubleSolenoid primaryOne;
  private DoubleSolenoid primaryTwo;
  private DoubleSolenoid secondaryOne;
  private DoubleSolenoid secondaryTwo;
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
  private boolean isPrimaryPistonEngaged;
  private boolean isSecondaryPistonEngaged;
  private boolean isSmartDashboardTestTimesShown;
  public String mode = ""; //auto, teleop, test

  public ActiveClimb() {
    winchMotor = new TalonFX(ActiveClimbConstants.FALCON_CAN_ID);
    //winchSecondaryMotor = new TalonFX(ClimbConstants.FALCON2_CAN_ID /*Available Number*/); //TODO:CHECK IF WE NEED MORE NUMBERS
    primaryOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
    primaryTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
    secondaryOne = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    secondaryTwo = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);
    
    armOneLimitOne = new DigitalInput(2);
    armOneLimitTwo = new DigitalInput(3);
    armOneLimitThree = new DigitalInput(4);
    armOneLimitFour = new DigitalInput(5);
    armTwoLimitOne = new DigitalInput(6);
    armTwoLimitTwo = new DigitalInput(7);
    armTwoLimitThree = new DigitalInput(8);
    armTwoLimitFour = new DigitalInput(9);

    armOneState = new String();
    armTwoState = new String();

    winchMotor.configFactoryDefault();
    // winchSecondaryMotor.configFactoryDefault();

    winchMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ActiveClimbConstants.SLOTIDX, ActiveClimbConstants.TIMEOUT_MS);
    winchMotor.config_kP(ActiveClimbConstants.SLOTIDX, ActiveClimbConstants.PID_MAIN[0], ActiveClimbConstants.TIMEOUT_MS);
    winchMotor.config_kI(ActiveClimbConstants.SLOTIDX, ActiveClimbConstants.PID_MAIN[1], ActiveClimbConstants.TIMEOUT_MS);
    winchMotor.config_kD(ActiveClimbConstants.SLOTIDX, ActiveClimbConstants.PID_MAIN[2], ActiveClimbConstants.TIMEOUT_MS);
    /*
    winchSecondaryMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, ClimbConstants.SLOTIDX, ClimbConstants.TIMEOUT_MS);
    winchSecondaryMotor.config_kP(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[0], ClimbConstants.TIMEOUT_MS);
    winchSecondaryMotor.config_kI(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[1], ClimbConstants.TIMEOUT_MS);
    winchSecondaryMotor.config_kD(ClimbConstants.SLOTIDX, ClimbConstants.PID_SECONDARY[2], ClimbConstants.TIMEOUT_MS);
    */
    winchMotor.configPeakOutputForward(.05, ActiveClimbConstants.TIMEOUT_MS); //TODO: TUNNNNNNNEEEEEEEEEEEEEEEEEEEEEEEE
    winchMotor.configPeakOutputReverse(-.05, ActiveClimbConstants.TIMEOUT_MS); //TODO: TUNNNNNNNEEEEEEEEEEEEEEEEEEEEEEEE
    /*
    winchSecondaryMotor.configPeakOutputForward(.05, ClimbConstants.TIMEOUT_MS);
    winchSecondaryMotor.configPeakOutputReverse(-.05, ClimbConstants.TIMEOUT_MS);
    */
    
    isPrimaryPistonEngaged = false;
    isSecondaryPistonEngaged = false;
    isSmartDashboardTestTimesShown = false;
  }

  /**
   * Adds Ten to the Position of the Winch Motor
   */

  public void winchManualUp() { //TODO: FIX WITH A RATE THAT I FIND
    winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition() + 10);
    // winchSecondaryMotor.set(ControlMode.Position, winchSecondaryMotor.getSelectedSensorPosition() + 10); //TODO: CHECK DIRECTIONS
  }

  /**
   * Subtracts Ten from the Position of the Winch Motor
   */

  public void winchManualDown() { //TODO: FIX WITH A RATE THAT I FIND
    winchMotor.set(ControlMode.Position, winchMotor.getSelectedSensorPosition() - 10);
    // winchSecondaryMotor.set(ControlMode.Position, winchSecondaryMotor.getSelectedSensorPosition() - 10); //TODO: CHECK DIRECTIONS
  }

  /**
   * Winch Up to the Max Position with 5% of the max speed
   */

  public void winchUp() {
    winchMotor.set(ControlMode.Position, ActiveClimbConstants.WINCH_MAX_LENGTH_POS); //TODO:CHANGE CONSTANT
    // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MAX_LENGTH_POS); //TODO: Add displacement from starting tic
  }

  /**
   * Winch Down to the Min Position with 5% of the max speed
   */

  public void winchDown() {
    winchMotor.set(ControlMode.Position, ActiveClimbConstants.WINCH_MIN_LENGTH_POS); //TODO:CHANGE CONSTANT
    // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MIN_LENGTH_POS); //TODO: Add displacement from starting tic
  }

  /**
   * Extend the Primary Pistons of Both Arm
   */

  public void primaryPistonsEngage() {
    primaryOne.set(Value.kForward);
    primaryTwo.set(Value.kForward);
    isPrimaryPistonEngaged = true;
  }

  /**
   * Retract the Primary Pistons of Both Arm
   */

  public void primaryPistonsRetract() {
    primaryOne.set(Value.kReverse);
    primaryTwo.set(Value.kReverse);
    isPrimaryPistonEngaged = false;
  }

  /**
   * Extend the Secondary Pistons of Both Arm
   */

  public void secondaryPistonsEngage() {
    secondaryOne.set(Value.kForward);
    secondaryTwo.set(Value.kForward);
    isSecondaryPistonEngaged = true;
  }

  /**
   * Retract the Secondary Pistons of Both Arm
   */

  public void secondaryPistonsRetract() {
    secondaryOne.set(Value.kReverse);
    secondaryTwo.set(Value.kReverse);
    isSecondaryPistonEngaged = false;
  }

  /**
   * Gets the State of Arm One by evaluating the state of all four limit switches
   */

  public void getStageOfArmOne() {
    armOneState = "";
    if(getArmOneLimitOne()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitTwo()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitThree()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitFour()) {armOneState.concat("1");} else {armOneState.concat("0");}
  }

  /**
   * Gets the State of Arm Two by evaluating the state of all four limit switches
   */

  public void getStageOfArmTwo() {
    armTwoState = "";
    if(getArmTwoLimitOne()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitTwo()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitThree()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitFour()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
  }

  /**Returns the State of Arm One gathered by getStateOfArmOne()*/
  public String getArmOneState() {return armOneState;}
  /**Returns the State of Arm Two gathered by getStateOfArmOne()*/
  public String getArmTwoState() {return armTwoState;}
  /**Returns the State of the Primary Pistons*/
  public boolean getPrimaryPistonEngaged() {return isPrimaryPistonEngaged;}
  /**Returns the State of the Secondary Pistons*/
  public boolean getSecondaryPistonEngaged() {return isSecondaryPistonEngaged;}

  public boolean getArmOneLimitOne() {return armOneLimitOne.get();}
  public boolean getArmOneLimitTwo() {return armOneLimitTwo.get();}
  public boolean getArmOneLimitThree() {return armOneLimitThree.get();}
  public boolean getArmOneLimitFour() {return armOneLimitFour.get();}
  public boolean getArmTwoLimitOne() {return armTwoLimitOne.get();}
  public boolean getArmTwoLimitTwo() {return armTwoLimitTwo.get();}
  public boolean getArmTwoLimitThree() {return armTwoLimitThree.get();}
  public boolean getArmTwoLimitFour() {return armTwoLimitFour.get();}

  /**
   * Puts Tiles on Shuffleboard (by SmartDashboard) to use in TestPeriodic
   */

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putNumber("Winch Falcon PCT", 0);
    // SmartDashboard.putNumber("Secondary Winch Falcon PCT", 0);
    isSmartDashboardTestTimesShown = true;
  }

  /**
   * In TestPeriodic constantly read the values in Shuffleboard
   */

  public void setValuesFromSmartDashboard() {
    winchMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Winch Falcon PCT", 0));
    // wincSecondaryMotor.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Secondary Winch Falcon PCT", 0));
  }

  @Override
  public void periodic() {
    if(!isSmartDashboardTestTimesShown) putSmartDashboardControlCommands();
    if(mode.equals("test")) setValuesFromSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {}
}