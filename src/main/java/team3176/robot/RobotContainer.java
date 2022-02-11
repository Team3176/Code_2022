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
import team3176.robot.commands.auton.*;
import team3176.robot.commands.climbActive.*;
import team3176.robot.commands.climbPassive.*;
import team3176.robot.commands.common.*;
import team3176.robot.commands.teleop.*;
import team3176.robot.constants.*;
import team3176.robot.subsystems.ExampleSubsystem;
import team3176.robot.subsystems.climb.*;
import team3176.robot.subsystems.controller.*;
import team3176.robot.subsystems.drivetrain.*;
import team3176.robot.subsystems.indexer.*;
import team3176.robot.subsystems.intake.*;
import team3176.robot.subsystems.shooter.*;
import team3176.robot.subsystems.vision.*;
import team3176.robot.util.Joystick.*;
import team3176.robot.util.PowerManagement.*;
import team3176.robot.util.XboxController.*;
import team3176.robot.subsystems.drivetrain.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Intake m_Intake = Intake.getInstance();
  private Controller m_Controller;
  private Compressor m_Compressor;
  private Drivetrain m_Drivetrain;
  // The robot's subsystems and commands are defined here...

  private ClimbActive m_ClimbActive;
  private ClimbPassive m_ClimbPassive;
  private final Vision m_Vision = Vision.getInstance();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();


  private final Angler m_Angler = Angler.getInstance();
  private final Indexer m_Indexer = Indexer.getInstance();
  // private final Transfer m_Transfer = Transfer.getInstance();
  // private final Flywheel m_Flywheel = Flywheel.getInstance();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Command m_AnglerShuffleboardTest = new AnglerShuffleboardTest();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    // m_Intake = Intake.getInstance();
    // m_Intake = new Intake(new IntakeIO() {});
    m_Controller = Controller.getInstance();
    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    m_Compressor.enableDigital();
    if ( MasterConstants.ISCLIMBPASSIVE ) {
      m_ClimbPassive = ClimbPassive.getInstance();
    } else {
      m_ClimbActive = ClimbActive.getInstance();
    }
    

    // Configure the button bindings
    configureButtonBindings();

    m_Drivetrain = Drivetrain.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDrive(
      () -> m_Controller.getForward(), 
      () -> m_Controller.getStrafe(),
      () -> m_Controller.getSpin()
      //() -> m_Controller.isFieldCentricButtonPressed(),
      //() -> m_Controller.isRobotCentricButtonPressed()
    ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_Controller.getOp_A().whenActive(new I2CTest());
    m_Controller.getOp_X().whenActive(new ExtendIntake());
    m_Controller.getOp_Y().whenActive(new RetractIntake());
    m_Controller.getOp_A().whenActive(new IntakeSpin());
    m_Controller.getOp_B().whenActive(new IntakeSpint());

    // m_Angler.setDefaultCommand(m_AnglerShuffleboardTest);

    m_Controller.getOp_A().whenActive(new SwitchVisionPipeline(m_Vision));
    m_Controller.getOp_B().whenActive(new SwitchVisionMode(m_Vision));
    m_Controller.getOp_Y().whenActive(new CalculateCameraTargetDistance(m_Vision));
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
