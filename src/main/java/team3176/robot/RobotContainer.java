// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;       //TODO: SO UNDERLINE -- DO NOT OVERWRITE --

import team3176.robot.constants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.commands.auton.*;
import team3176.robot.commands.common.*;
import team3176.robot.commands.teleop.*;
import team3176.robot.commands.climbActive.*;
import team3176.robot.commands.climbPassive.*;
import team3176.robot.subsystems.controller.*;
import team3176.robot.subsystems.drivetrain.*;
import team3176.robot.subsystems.indexer.*;
import team3176.robot.subsystems.intake.*;
import team3176.robot.subsystems.shooter.*;
import team3176.robot.subsystems.vision.*;
import team3176.robot.subsystems.climb.*;

import edu.wpi.first.wpilibj.GenericHID; //TODO: SEE IF WE NEED THESE IMPORTS
import edu.wpi.first.wpilibj.XboxController;
import team3176.robot.constants.*;
import team3176.robot.util.Joystick.*;
import team3176.robot.util.PowerManagement.*;
import team3176.robot.util.XboxController.*;
import team3176.robot.subsystems.ExampleSubsystem;
import team3176.robot.commands.ExampleCommand;

public class RobotContainer {

  private final Intake m_Intake;
  private final Controller m_Controller;
  private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  private final Vision m_Vision;
  private final Angler m_Angler;
  private final Transfer m_Transfer;
  private final Flywheel m_Flywheel;
  private final Indexer m_Indexer;

  private ClimbActive m_ClimbActive;
  private ClimbPassive m_ClimbPassive;

  private SendableChooser<String> m_autonChooser;
  private static final String m_autoOneRenameAfterAssigned = "s_optionOneRenameAlso";

  private final Command m_AnglerShuffleboardTest = new AnglerShuffleboardTest(); //TODO: GET RID OF THIS and INVESTIGATE


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    // m_Intake = Intake.getInstance();
    // m_Intake = new Intake(new IntakeIO() {});
    m_Controller = Controller.getInstance();
    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    // m_Compressor.enableDigital();
    m_Compressor.disable();
    // if ( MasterConstants.ISCLIMBPASSIVE) {
    //   m_ClimbPassive = ClimbPassive.getInstance();
    // } else {
    //   m_ClimbActive = ClimbActive.getInstance();
    // }  
    m_Indexer = Indexer.getInstance();
    m_Intake = Intake.getInstance();
    m_Vision = Vision.getInstance();
    m_Angler = Angler.getInstance();
    m_Flywheel = Flywheel.getInstance();
    m_Transfer = Transfer.getInstance();

    // if (MasterConstants.ISCLIMBPASSIVE) {
    //   m_ClimbPassive = ClimbPassive.getInstance();
    // } else {
    //   m_ClimbActive = ClimbActive.getInstance();
    // }

    //m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    //m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    
    m_Drivetrain = Drivetrain.getInstance();
    if (!MasterConstants.IS_TUNING_MODE) { 
      m_Drivetrain.setDefaultCommand(new SwerveDrive(
        () -> m_Controller.getForward(), 
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin()
        //() -> m_Controller.isFieldCentricButtonPressed(),
        //() -> m_Controller.isRobotCentricButtonPressed()
        ));
    } else {
      m_Drivetrain.setDefaultCommand(new SwerveDriveTune());
    }

    m_autonChooser = new SendableChooser<>(); //TODO: Put them in the order of frequency that they will be used
    m_autonChooser.addOption("Auto: Rename This Version that should display understandably", m_autoOneRenameAfterAssigned);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
  }

  
  private void configureButtonBindings() {
    // m_Controller.getOp_X().whenActive(new IntakeMotorToggle());
    // m_Controller.getOp_Y().whenActive(new IntakePistonToggle());
    // m_Controller.getOp_Start().whenActive(new TransferToggle());
    // m_Controller.getOp_Back().whenActive(new ShooterReset());

    // m_Controller.getOp_A_FS().whenActive(new IndexerForward());
    // m_Controller.getOp_B_FS().whenActive(new IndexerBack());
    // m_Controller.getOp_X_FS().whenActive(new FlywheelVelocityToggle());
    // m_Controller.getOp_Y_FS().whenActive(new TransferToggle());
    // m_Controller.getOp_Start_FS().whenActive(new ShootWithoutTarget());
    // m_Controller.getOp_Back_FS().whenActive(new ShootReset());

    // m_Controller.getOp_A_DS().whenActive(new WinchUp());
    // m_Controller.getOp_B_DS().whenActive(new PrimaryPistonToggle());
    // m_Controller.getOp_X_DS().whenActive(new SecondaryPistonToggle());
    // m_Controller.getOp_Y_DS().whenActive(new WinchDown());
    // m_Controller.getOp_Start_DS().whenActive(new ClimbDisableToggle());
    // m_Controller.getOp_Back_DS().whenActive(new ShootReset());



    // m_Controller.getOp_RightTrigger().whenActive(new ShootSequence());
    // m_Controller.getOp_LeftTrigger().whenActive(new FlywheelToggle());
    // m_Controller.getOp_DPAD_RIGHT().whenActive(new ClimbDisableToggle());

    // Active Climb
    // m_Controller.getOp_DPAD_UP().whenActive(new ClimbToMid());
    // m_Controller.getOp_DPAD_DOWN().whenActive(new ClimbToTraversal());
    // m_Controller.getOp_DPAD_LEFT().whenActive(new ClimbToHigh());
    // Passive Climb
    // m_Controller.getOp_DPAD_UP().whenActive(new ClimbExtend());
    // m_Controller.getOp_DPAD_DOWN().whenActive(new ClimbRetract());

    // m_Angler.setAngle(m_Controller.getOp_LeftY());


    // m_Angler.setDefaultCommand(m_AnglerShuffleboardTest); //TODO: INVESTIGATE

    m_Controller.getOp_A().whenActive(new SwitchVisionPipeline(m_Vision));
    m_Controller.getOp_B().whenActive(new SwitchVisionMode(m_Vision));
    m_Controller.getOp_Y().whenActive(new CalculateCameraTargetDistance(m_Vision));
    m_Controller.getOp_X().whenActive(new CalculateTargetDistance(m_Vision));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    String chosen = m_autonChooser.getSelected();
    if(chosen.equals(m_autoOneRenameAfterAssigned)) return new auto1Hypo(); //TODO: Remove Autos that were game/strategy specific but more would be the same line but else if because efficient even though its not






    return new auto1Hypo(); //TODO: Also command should still not exist but I want to get rid of example command but this return should be the most common
  }
}