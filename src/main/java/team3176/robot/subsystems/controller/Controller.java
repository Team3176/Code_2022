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
import team3176.robot.util.XboxDBLShift.*;

public class Controller {

  private static Controller instance = new Controller();
  public Controller getInstance() {return instance;}

  private final Joystick transStick;
  private final Joystick rotStick;
  private final XboxController operator;

  private final JoystickButton transStick_Button1;
  private final JoystickButton transStick_Button2;
  private final JoystickButton transStick_Button3;
  private final JoystickButton transStick_Button4;
  private final JoystickButton transStick_Button5;
  private final JoystickButton transStick_Button6;
  private final JoystickButton transStick_Button7;
  private final JoystickButton transStick_Button8;
  private final JoystickButton transStick_Button9;
  private final JoystickButton transStick_Button10;
  private final JoystickButton transStick_Button11;
  private final JoystickButton transStick_Button12;
  private final JoystickButton transStick_Button13;
  private final JoystickButton transStick_Button14;
  private final JoystickButton transStick_Button15;
  private final JoystickButton transStick_Button16;

  private final JoystickButton rotStick_Button1;
  private final JoystickButton rotStick_Button2;
  private final JoystickButton rotStick_Button3;
  private final JoystickButton rotStick_Button4;
  private final JoystickButton rotStick_Button5;
  private final JoystickButton rotStick_Button6;
  private final JoystickButton rotStick_Button7;
  private final JoystickButton rotStick_Button8;
  private final JoystickButton rotStick_Button9;
  private final JoystickButton rotStick_Button10;
  private final JoystickButton rotStick_Button11;
  private final JoystickButton rotStick_Button12;
  private final JoystickButton rotStick_Button13;
  private final JoystickButton rotStick_Button14;
  private final JoystickButton rotStick_Button15;
  private final JoystickButton rotStick_Button16;

  //Add stick and slider and axes (how to say multiple axises)

  private final Trigger op_A;
  private final Trigger op_A_Shift;
  private final Trigger op_A_Double_Shift;
  private final Trigger op_B;
  private final Trigger op_B_Shift;
  private final Trigger op_B_Double_Shift;
  private final Trigger op_X;
  private final Trigger op_X_Shift;
  private final Trigger op_X_Double_Shift;
  private final Trigger op_Y;
  private final Trigger op_Y_Shift;
  private final Trigger op_Y_Double_Shift;
  private final Trigger op_Start;
  private final Trigger op_Start_Shift;
  private final Trigger op_Start_Double_Shift;
  private final Trigger op_Back;
  private final Trigger op_Back_Shift;
  private final Trigger op_Back_Double_Shift;
  private final Trigger op_LTrigger; //TODO: SEE IF WE WANT SHIFTED TRIGGERS
  private final Trigger op_RTrigger; //TODO: SEE IF WE WANT SHIFTED TRIGGERS

  private final POVButton op_DPAD_Up;
  private final POVButton op_DPAD_Left;
  private final POVButton op_DPAD_Down;
  private final POVButton op_DPAD_Right;

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

    op_A = new XboxMain(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_A_Shift = new XboxShift(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_A_Double_Shift = new XboxDBLShift(operator, Button.kA.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_B = new XboxMain(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_B_Shift = new XboxShift(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_B_Double_Shift = new XboxDBLShift(operator, Button.kB.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_X = new XboxMain(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_X_Shift = new XboxShift(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_X_Double_Shift = new XboxDBLShift(operator, Button.kX.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Y = new XboxMain(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Y_Shift = new XboxShift(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Y_Double_Shift = new XboxDBLShift(operator, Button.kY.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Start = new XboxMain(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Start_Shift = new XboxShift(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Start_Double_Shift = new XboxDBLShift(operator, Button.kStart.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Back = new XboxMain(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Back_Shift = new XboxShift(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_Back_Double_Shift = new XboxDBLShift(operator, Button.kBack.value, Button.kLeftBumper.value, Button.kRightBumper.value);
    op_LTrigger = new XboxAxisAsButton(operator, Axis.kLeftTrigger.value, ControllerConstants.TRIGGER_THRESHOLD); //TODO: CHANGE THRESHOLD
    op_RTrigger = new XboxAxisAsButton(operator, Axis.kRightTrigger.value, ControllerConstants.TRIGGER_THRESHOLD);
    
    op_DPAD_Up = new POVButton(operator, 0);
    op_DPAD_Right = new POVButton(operator, 90);
    op_DPAD_Down = new POVButton(operator, 180);
    op_DPAD_Left = new POVButton(operator, 270);
  }

  // public double getForward() {
  //   if(Math.abs(-transStick.getY() < 0.06)) return 0.0;
  //   return 
  // }

  public JoystickButton getTransStick_Button1() {return transStick_Button1;}
  public JoystickButton getTransStick_Button2() {return transStick_Button2;}
  public JoystickButton getTransStick_Button3() {return transStick_Button3;}
  public JoystickButton getTransStick_Button4() {return transStick_Button4;}
  public JoystickButton getTransStick_Button5() {return transStick_Button5;}
  public JoystickButton getTransStick_Button6() {return transStick_Button6;}
  public JoystickButton getTransStick_Button7() {return transStick_Button7;}
  public JoystickButton getTransStick_Button8() {return transStick_Button8;}
  public JoystickButton getTransStick_Button9() {return transStick_Button9;}
  public JoystickButton getTransStick_Button10() {return transStick_Button10;}
  public JoystickButton getTransStick_Button11() {return transStick_Button11;}
  public JoystickButton getTransStick_Button12() {return transStick_Button12;}
  public JoystickButton getTransStick_Button13() {return transStick_Button13;}
  public JoystickButton getTransStick_Button14() {return transStick_Button14;}
  public JoystickButton getTransStick_Button15() {return transStick_Button15;}
  public JoystickButton getTransStick_Button16() {return transStick_Button16;}

  public JoystickButton getrotStick_Button1() {return rotStick_Button1;}
  public JoystickButton getrotStick_Button2() {return rotStick_Button2;}
  public JoystickButton getrotStick_Button3() {return rotStick_Button3;}
  public JoystickButton getrotStick_Button4() {return rotStick_Button4;}
  public JoystickButton getrotStick_Button5() {return rotStick_Button5;}
  public JoystickButton getrotStick_Button6() {return rotStick_Button6;}
  public JoystickButton getrotStick_Button7() {return rotStick_Button7;}
  public JoystickButton getrotStick_Button8() {return rotStick_Button8;}
  public JoystickButton getrotStick_Button9() {return rotStick_Button9;}
  public JoystickButton getrotStick_Button10() {return rotStick_Button10;}
  public JoystickButton getrotStick_Button11() {return rotStick_Button11;}
  public JoystickButton getrotStick_Button12() {return rotStick_Button12;}
  public JoystickButton getrotStick_Button13() {return rotStick_Button13;}
  public JoystickButton getrotStick_Button14() {return rotStick_Button14;}
  public JoystickButton getrotStick_Button15() {return rotStick_Button15;}
  public JoystickButton getrotStick_Button16() {return rotStick_Button16;}

  public Trigger getOp_A() {return op_A;}
  public Trigger getOp_A_FS() {return op_A_Shift;}
  public Trigger getOp_A_DS() {return op_A_Double_Shift;}
  public Trigger getOp_B() {return op_B;}
  public Trigger getOp_B_FS() {return op_B_Shift;}
  public Trigger getOp_B_DS() {return op_B_Double_Shift;}
  public Trigger getOp_X() {return op_X;}
  public Trigger getOp_X_FS() {return op_X_Shift;}
  public Trigger getOp_X_DS() {return op_X_Double_Shift;}
  public Trigger getOp_Y() {return op_Y;}
  public Trigger getOp_Y_FS() {return op_Y_Shift;}
  public Trigger getOp_Y_DS() {return op_Y_Double_Shift;}
  public Trigger getOp_Start() {return op_Start;}
  public Trigger getOp_Start_FS() {return op_Start_Shift;}
  public Trigger getOp_Start_DS() {return op_Start_Double_Shift;}
  public Trigger getOp_Back() {return op_Back;}
  public Trigger getOp_Back_FS() {return op_Back_Shift;}
  public Trigger getOp_Back_DS() {return op_Back_Double_Shift;}
  public Trigger getOp_LeftTrigger() {return op_LTrigger;}
  public Trigger getOp_RightTrigger() {return op_RTrigger;}
  
  public POVButton getOp_DPAD_UP() {return op_DPAD_Up;}
  public POVButton getOp_DPAD_RIGHT() {return op_DPAD_Right;}
  public POVButton getOp_DPAD_DOWN() {return op_DPAD_Down;}
  public POVButton getOp_DPAD_LEFT() {return op_DPAD_Left;}
}
