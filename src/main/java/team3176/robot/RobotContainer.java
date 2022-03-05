// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import team3176.robot.constants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.subsystems.*;
import team3176.robot.subsystems.drivetrain.*;
import team3176.robot.subsystems.Vision;

import team3176.robot.commands.Climb.*;
import team3176.robot.commands.CMD_Groups.*;
// import team3176.robot.commands.Drivetrain.*;
import team3176.robot.commands.Drivetrain.imported.*;
import team3176.robot.commands.Indexer.*;
import team3176.robot.commands.Intake.*;
import team3176.robot.commands.Shooter.*;
import team3176.robot.commands.Test.*;
import team3176.robot.commands.Util.*;
import team3176.robot.commands.Vision.*;

public class RobotContainer {

  private final Intake m_Intake;
  private final Controller m_Controller;
  private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  private final Vision m_Vision;
  private final Angler m_Angler;
  private final Feeder m_Transfer;
  private final Flywheel m_Flywheel;
  private final Indexer m_Indexer;
  private final Climb m_Climb;

  private SendableChooser<String> m_autonChooser;
  private static final String m_5 = "s_5BallAuto";
  private static final String m_4 = "s_4BallAuto";
  private static final String m_3 = "s_3BallAuto";
  private static final String m_2 = "s_2BallAuto";
  private static final String m_B = "s_Block";
  private static final String m_MS = "s_Move&Shoot";
  private static final String m_S = "s_Shoot";
  private static final String m_M = "s_ExitTarmac";
  private static final String m_D = "s_Move<Specify>in<Direction>";

  // private final Command m_AnglerShuffleboardTest = new AnglerShuffleboardTest(); //TODO: GET RID OF THIS and INVESTIGATE
  
  public RobotContainer() {
    m_Controller = Controller.getInstance();
    m_Indexer = Indexer.getInstance();
    m_Intake = Intake.getInstance();
    m_Vision = Vision.getInstance();
    m_Angler = Angler.getInstance();
    m_Flywheel = Flywheel.getInstance();
    m_Transfer = Feeder.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    m_Climb = Climb.getInstance();

    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START

    if (!MasterConstants.IS_TUNING_MODE) { 
      m_Drivetrain.setDefaultCommand(new SwerveDrive(
        () -> m_Controller.getForward(), 
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin()//,
        //() -> m_Controller.isFieldCentricButtonPressed(),
        //() -> m_Controller.isRobotCentricButtonPressed()
        ));
    } else {
      m_Drivetrain.setDefaultCommand(new SwerveDriveTune());
    }

    m_autonChooser = new SendableChooser<>(); //TODO: Put them in the order of frequency that they will be used
    m_autonChooser.addOption("Auto: 5-Ball Auto (Mission: Impossible)", m_5);
    m_autonChooser.addOption("Auto: 4-Ball Auto (Mission: Kinda Impossible)", m_4);
    m_autonChooser.addOption("Auto: 3-Ball Auto (Mission: Probable)", m_3);
    m_autonChooser.addOption("Auto: 2-Ball Auto (Mission: Feasible)", m_2);
    m_autonChooser.addOption("Auto: 1-Ball Auto (Mission: Undershoot)", m_MS);
    m_autonChooser.addOption("Auto: Sit Shoot (Mission: Bare Minimum)", m_S);
    m_autonChooser.addOption("Auto: ExitTarmac", m_M);
    m_autonChooser.addOption("Auto: Block", m_B);
    m_autonChooser.addOption("Auto: Move <dis> in <dir>", m_D);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
    System.out.println("init run");
  }

  
  private void configureButtonBindings() {
    m_Controller.getTransStick_Button4().whenActive(new ToggleCoordSys());
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

    // m_Controller.getOp_A().whenActive(new SwitchVisionPipeline(m_Vision));
    // m_Controller.getOp_B().whenActive(new SwitchVisionMode(m_Vision));
    // m_Controller.getOp_Y().whenActive(new CalculateCameraTargetDistance(m_Vision));
    m_Controller.getOp_A().whenActive(new ExtendIntake());
    m_Controller.getOp_B().whenActive(new RetractIntake());
    m_Controller.getOp_X().whenActive(new I2CTest());
  }

  public Command getAutonomousCommand() {
    System.out.println("run");

    String chosen = m_autonChooser.getSelected();
    if(chosen.equals(m_5)) return new IntakeSpin(); //TODO: Put in order of frequency so the bot doesn't have to process more (shouldn't effect anything but just good to have)
    if(chosen.equals(m_4)) return new IntakeSpin();
    if(chosen.equals(m_3)) return new IntakeSpin();
    if(chosen.equals(m_2)) return new IntakeSpin();
    if(chosen.equals(m_MS)) return new IntakeSpin();
    if(chosen.equals(m_S)) return new IntakeSpin();
    if(chosen.equals(m_M)) return new IntakeSpin();
    if(chosen.equals(m_B)) return new IntakeSpin();
    if(chosen.equals(m_D)) return new autoDis5Back();

    System.out.println(m_autonChooser.getSelected());
    System.out.println(m_D);
    System.out.println("run");

    return new IntakeSpin(); //TODO: Return the most common auton
  }
}