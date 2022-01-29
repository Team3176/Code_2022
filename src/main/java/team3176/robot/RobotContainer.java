// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.commands.ExampleCommand;
import team3176.robot.commands.teleop.ExtendIntake;
import team3176.robot.commands.teleop.IntakeSpin;
import team3176.robot.commands.teleop.IntakeSpint;
import team3176.robot.commands.teleop.RetractIntake;
import team3176.robot.subsystems.ExampleSubsystem;
import team3176.robot.subsystems.intake.*;
import team3176.robot.subsystems.controller.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Intake m_Intake;
  private Controller m_Controller;
  private Compressor m_Compressor;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_Intake = Intake.getInstance();
    m_Controller = Controller.getInstance();
    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    m_Compressor.enableDigital();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_Controller.getDuke_XButton().whenPressed(new ExtendIntake());
    m_Controller.getDuke_YButton().whenPressed(new RetractIntake());
    m_Controller.getDuke_AButton().whenPressed(new IntakeSpin());
    m_Controller.getDuke_BButton().whenPressed(new IntakeSpint());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
