// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3176.robot.constants.ControllerConstants;
import team3176.robot.util.XboxAxisAsButton;
import team3176.robot.util.XboxLoneButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;

public class Controller {
  private static Controller instance = new Controller();
  private XboxController duke;
  private JoystickButton xButton, yButton, aButton, bButton;

  public Controller() 
  {
    duke = new XboxController(2);
    xButton = new JoystickButton(duke, Button.kX.value);
    yButton = new JoystickButton(duke, Button.kY.value);
    aButton = new JoystickButton(duke, Button.kA.value);
    bButton = new JoystickButton(duke, Button.kB.value);
  }

  public JoystickButton getDuke_XButton() 
  {
    return xButton;
  }

  public JoystickButton getDuke_YButton() 
  {
    return yButton;
  }

  public JoystickButton getDuke_AButton() 
  {
    return aButton;
  }

  public JoystickButton getDuke_BButton() 
  {
    return bButton;
  }

  public static Controller getInstance() 
  {
    return instance;
  }

}
