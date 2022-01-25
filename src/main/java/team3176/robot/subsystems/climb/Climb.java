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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ClimbConstants;

public class Climb extends SubsystemBase {
  private static Climb instance = new Climb();
  public static Climb getInstance() {return instance;}

  TalonFX winchMotor;
  //TalonFX winchSecondaryMotor;
  DoubleSolenoid primaryOne;
  DoubleSolenoid primaryTwo;
  DoubleSolenoid secondaryOne;
  DoubleSolenoid secondaryTwo;
  DigitalInput armOneLimitOne;
  DigitalInput armOneLimitTwo;
  DigitalInput armOneLimitThree;
  DigitalInput armOneLimitFour;
  DigitalInput armTwoLimitOne;
  DigitalInput armTwoLimitTwo;
  DigitalInput armTwoLimitThree;
  DigitalInput armTwoLimitFour;
  String armOneState;
  String armTwoState;

  public Climb() {
    winchMotor = new TalonFX(ClimbConstants.FALCON_CAN_ID);
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
  }

  //TODO: CREATE A JOYSTICK AXIS CONTROLLED WINCH

  public void winchUp() {
    winchMotor.set(ControlMode.Position, ClimbConstants.WINCH_MAX_LENGTH_POS); //TODO:CHANGE CONSTANT
    // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MAX_LENGTH_POS); //TODO: Add displacement from starting tic
  }

  public void winchDown() {
    winchMotor.set(ControlMode.Position, ClimbConstants.WINCH_MIN_LENGTH_POS); //TODO:CHANGE CONSTANT
    // winchSecondaryMotor.set(ControlMode.Position, ClimbConstants.WINCH_MIN_LENGTH_POS); //TODO: Add displacement from starting tic
  }

  public void primaryPistonsEngage() {
    primaryOne.set(Value.kForward);
    primaryTwo.set(Value.kForward);
  }

  public void primaryPistonsRetract() {
    primaryOne.set(Value.kReverse);
    primaryTwo.set(Value.kReverse);
  }

  public void secondaryPistonsEngage() {
    secondaryOne.set(Value.kForward);
    secondaryTwo.set(Value.kForward);
  }

  public void secondaryPistonsRetract() {
    secondaryOne.set(Value.kReverse);
    secondaryTwo.set(Value.kReverse);
  }

  public void getStageOfArmOne() {
    armOneState = "";
    if(getArmOneLimitOne()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitTwo()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitThree()) {armOneState.concat("1");} else {armOneState.concat("0");}
    if(getArmOneLimitFour()) {armOneState.concat("1");} else {armOneState.concat("0");}
  }

  public void getStageOfArmTwo() {
    armTwoState = "";
    if(getArmTwoLimitOne()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitTwo()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitThree()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
    if(getArmTwoLimitFour()) {armTwoState.concat("1");} else {armTwoState.concat("0");}
  }

  public String getArmOneState() {return armOneState;}
  public String getArmTwoState() {return armTwoState;}

  public boolean getArmOneLimitOne() {return armOneLimitOne.get();}
  public boolean getArmOneLimitTwo() {return armOneLimitTwo.get();}
  public boolean getArmOneLimitThree() {return armOneLimitThree.get();}
  public boolean getArmOneLimitFour() {return armTwoLimitFour.get();}
  public boolean getArmTwoLimitOne() {return armTwoLimitOne.get();}
  public boolean getArmTwoLimitTwo() {return armTwoLimitTwo.get();}
  public boolean getArmTwoLimitThree() {return armTwoLimitThree.get();}
  public boolean getArmTwoLimitFour() {return armTwoLimitFour.get();}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}