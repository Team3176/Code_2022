// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3176.robot.constants.ControllerConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import team3176.robot.util.*;

public class Controller {

  Controller instance = new Controller();
  public Controller getInstance() {return instance;}

  Joystick transStick;
  Joystick rotStick;
  XboxController operator;

  JoystickButton transStick_Button1;
  JoystickButton transStick_Button2;
  JoystickButton transStick_Button3;
  JoystickButton transStick_Button4;
  JoystickButton transStick_Button5;
  JoystickButton transStick_Button6;
  JoystickButton transStick_Button7;
  JoystickButton transStick_Button8;
  JoystickButton transStick_Button9;
  JoystickButton transStick_Button10;
  JoystickButton transStick_Button11;
  JoystickButton transStick_Button12;
  JoystickButton transStick_Button13;
  JoystickButton transStick_Button14;
  JoystickButton transStick_Button15;
  JoystickButton transStick_Button16;

  JoystickButton rotStick_Button1;
  JoystickButton rotStick_Button2;
  JoystickButton rotStick_Button3;
  JoystickButton rotStick_Button4;
  JoystickButton rotStick_Button5;
  JoystickButton rotStick_Button6;
  JoystickButton rotStick_Button7;
  JoystickButton rotStick_Button8;
  JoystickButton rotStick_Button9;
  JoystickButton rotStick_Button10;
  JoystickButton rotStick_Button11;
  JoystickButton rotStick_Button12;
  JoystickButton rotStick_Button13;
  JoystickButton rotStick_Button14;
  JoystickButton rotStick_Button15;
  JoystickButton rotStick_Button16;

  //Add stick and slider and axes (how to say multiple axises)

  Trigger op_A;
  Trigger op_A_Shift;
  // Trigger op_A_DBLShift;
  Trigger op_B;
  Trigger op_B_Shift;
  // Trigger op_B_DBLShift;
  Trigger op_X;
  Trigger op_X_Shift;
  // Trigger op_X_DBLShift;
  Trigger op_Y;
  Trigger op_Y_Shift;
  // Trigger op_Y_DBLShift;
  Trigger op_Start;
  Trigger op_Start_Shift;
  // Trigger op_Start_DBLShift;
  Trigger op_Back;
  Trigger op_Back_Shift;
  // Trigger op_Back_DBLShift;
  Trigger op_LTrigger;
  Trigger op_LTrigger_Shift;
  // Trigger op_LTrigger_DBLShift;
  Trigger op_RTrigger;
  Trigger op_RTrigger_Shift;
  // Trigger op_RTrigger_DBLShift;
  
  
  //d-pad and bumpers

  public Controller() {
    transStick = new Joystick(ControllerConstants.TRANS_ID);
    rotStick = new Joystick(ControllerConstants.ROT_ID);
    operator = new XboxController(ControllerConstants.OP_ID);

    transStick_Button1 = new JoystickButton(transStick, 1);
    transStick_Button2 = new JoystickButton(transStick, 2);
    transStick_Button3 = new JoystickButton(transStick, 3);
    transStick_Button4 = new JoystickButton(transStick, 4);
    transStick_Button5 = new JoystickButton(transStick, 5);
    transStick_Button6 = new JoystickButton(transStick, 6);
    transStick_Button7 = new JoystickButton(transStick, 7);
    transStick_Button8 = new JoystickButton(transStick, 8);
    transStick_Button9 = new JoystickButton(transStick, 9);
    transStick_Button10 = new JoystickButton(transStick, 10);
    transStick_Button11 = new JoystickButton(transStick, 11);
    transStick_Button12 = new JoystickButton(transStick, 12);
    transStick_Button13 = new JoystickButton(transStick, 13);
    transStick_Button14 = new JoystickButton(transStick, 14);
    transStick_Button15 = new JoystickButton(transStick, 15);
    transStick_Button16 = new JoystickButton(transStick, 16);

    
    rotStick_Button1 = new JoystickButton(rotStick, 1);
    rotStick_Button2 = new JoystickButton(rotStick, 2);
    rotStick_Button3 = new JoystickButton(rotStick, 3);
    rotStick_Button4 = new JoystickButton(rotStick, 4);
    rotStick_Button5 = new JoystickButton(rotStick, 5);
    rotStick_Button6 = new JoystickButton(rotStick, 6);
    rotStick_Button7 = new JoystickButton(rotStick, 7);
    rotStick_Button8 = new JoystickButton(rotStick, 8);
    rotStick_Button9 = new JoystickButton(rotStick, 9);
    rotStick_Button10 = new JoystickButton(rotStick, 10);
    rotStick_Button11 = new JoystickButton(rotStick, 11);
    rotStick_Button12 = new JoystickButton(rotStick, 12);
    rotStick_Button13 = new JoystickButton(rotStick, 13);
    rotStick_Button14 = new JoystickButton(rotStick, 14);
    rotStick_Button15 = new JoystickButton(rotStick, 15);
    rotStick_Button16 = new JoystickButton(rotStick, 16);

    op_A = new XboxLoneButton(operator, Button.kA.value, Button.kLeftBumper.value);
    // op_A_Shift = new DoubleButton(operator, Button.kA.value, Button.kLeftBumper.value);
    op_B = new XboxLoneButton(operator, Button.kB.value, Button.kLeftBumper.value);
    op_X = new XboxLoneButton(operator, Button.kX.value, Button.kLeftBumper.value);
    op_Y = new XboxLoneButton(operator, Button.kY.value, Button.kLeftBumper.value);
    op_Start = new XboxLoneButton(operator, Button.kStart.value, Button.kLeftBumper.value);
    op_Back = new XboxLoneButton(operator, Button.kBack.value, Button.kLeftBumper.value);
    op_LTrigger = new XboxAxisAsButton(operator, Axis.kLeftTrigger.value, ControllerConstants.TRIGGER_THRESHOLD); //TODO: CHANGE THRESHOLD
    op_RTrigger = new XboxAxisAsButton(operator, Axis.kRightTrigger.value, ControllerConstants.TRIGGER_THRESHOLD);
    /*
    
  JoystickButton op_A_Shift;
  // JoystickButton op_A_DBLShift;
  JoystickButton op_B_Shift;
  // JoystickButton op_B_DBLShift;
  JoystickButton op_X_Shift;
  // JoystickButton op_X_DBLShift;
  
  JoystickButton op_Y_Shift;
  // JoystickButton op_Y_DBLShift;
  JoystickButton op_Start_Shift;
  // JoystickButton op_Start_DBLShift;
  JoystickButton op_Back_Shift;
  // JoystickButton op_Back_DBLShift;
  Trigger op_LTrigger_Shift;
  // Trigger op_LTrigger_DBLShift;
  Trigger op_RTrigger_Shift;
  // Trigger op_RTrigger_DBLShift;
    */


    
  }
}
