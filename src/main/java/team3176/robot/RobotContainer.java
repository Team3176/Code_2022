// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot;

import team3176.robot.constants.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import team3176.robot.subsystems.*;
import team3176.robot.subsystems.drivetrain.*;
import team3176.robot.subsystems.Vision;

import team3176.robot.commands.Climb.*;
import team3176.robot.commands.Auton.Auton1Ball;
import team3176.robot.commands.Auton.Auton2Balls;
import team3176.robot.commands.Auton.Auton3Balls;
import team3176.robot.commands.Auton.Auton4Balls;
import team3176.robot.commands.Auton.Auton5Balls;
import team3176.robot.commands.Auton.AutonBlock;
import team3176.robot.commands.Auton.AutonExitTarmac;
import team3176.robot.commands.Auton.AutonInstantShoot;
import team3176.robot.commands.Auton.AutonMove;
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

  private final PowerDistribution m_PDH;
  private final Intake m_Intake;
  private final Controller m_Controller;
  private final Compressor m_Compressor;
  private final Drivetrain m_Drivetrain;
  private final CoordSys m_CoordSys;
  private final Vision m_Vision;
  private final Angler m_Angler;
  private final Feeder m_Feeder;
  private final Flywheel m_Flywheel;
  private final Indexer m_Indexer;
  private final Climb m_Climb;
  private final Clarke m_Clarke;
  private SendableChooser<String> m_autonChooser;
  private static final String m_5 = "s_5BallAuto";
  private static final String m_4 = "s_4BallAuto";
  private static final String m_3 = "s_3BallAuto";
  private static final String m_2 = "s_2BallAuto";
  private static final String m_B = "s_Block";
  private static final String m_MS = "s_Move&Shoot";
  private static final String m_S = "s_Shoot";
  private static final String m_M = "s_ExitTarmac";
  private static final String m_6L = "s_Move6inToTheLeft";
  private static final String m_6R = "s_Move6inToTheRight";
  private static final String m_6F = "s_Move6inToTheFront";
  private static final String m_6B = "s_Move6inToTheBack";
  
  public RobotContainer() {
    m_Controller = Controller.getInstance();
    m_Indexer = Indexer.getInstance();
    m_Intake = Intake.getInstance();
    m_Vision = Vision.getInstance();
    m_Angler = Angler.getInstance();
    m_Flywheel = Flywheel.getInstance();
    m_Feeder = Feeder.getInstance();
    m_Drivetrain = Drivetrain.getInstance();
    m_CoordSys = CoordSys.getInstance();
    m_Climb = Climb.getInstance();
    m_Clarke = Clarke.getInstance();

    m_PDH = new PowerDistribution(1, ModuleType.kRev);
    m_PDH.clearStickyFaults();

    m_Compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    //TODO: ADD A WAY TO CLEAR STICKY FAULTS
    // m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    m_Compressor.enableDigital();

    // m_Indexer.setDefaultCommand(new Index());
    
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
    m_autonChooser.addOption("Auto: Move 6in Left", m_6L);
    m_autonChooser.addOption("Auto: Move 6in Right", m_6R);
    m_autonChooser.addOption("Auto: Move 6in Forward", m_6F);
    m_autonChooser.addOption("Auto: Move 6in Backwards", m_6B);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
    System.out.println("init run");
  }

  
  private void configureButtonBindings() {
    m_Controller.getTransStick_Button1().whenHeld(new SwerveTurboOn());
    m_Controller.getTransStick_Button1().whenReleased(new SwerveTurboOff());
    m_Controller.getTransStick_Button2().whenHeld(new SwerveDefense());
    m_Controller.getTransStick_Button4().whenPressed(new ToggleCoordSys());

    m_Controller.getRotStick_Button3().whenPressed(new ToggleSpinLock());
    m_Controller.getRotStick_Button4().whenPressed(new SwerveResetGyro());
    m_Controller.getRotStick_Button5().whenPressed(new SwervePodsAzimuthGoHome());

    // m_Controller.getOp_A().whileActiveOnce(new Intaking());
    // m_Controller.getOp_A().whenActive(new IntakeIndexerIntegration());
    // m_Controller.getOp_X().whenActive(new IntakeMotorToggle());
    // m_Controller.getOp_A().whileActiveOnce(new ShootToggleTest());
    // m_Controller.getOp_X().whileActiveOnce(new FlywheelToggleTest());
    // m_Controller.getOp_Y().whenActive(new IntakePistonToggle());
    // m_Controller.getOp_Start_DS().whileActiveOnce(new Intaking());

    m_Controller.getOp_A().whileActiveOnce(new Intaking());
    m_Controller.getOp_X().whenActive(new FlywheelPCT());
    m_Controller.getOp_Y().whileActiveOnce(new ShootPCT());
    m_Controller.getOp_B().whenActive(new FlywheelStop());
    
    m_Controller.getOp_Start().whenActive(new FeederToggle());
    m_Controller.getOp_Back().whenActive(new ShootReset());

    m_Controller.getOp_A_FS().whenActive(new IndexerHoldingMode());
    m_Controller.getOp_B_FS().whenActive(new IndexerLoadingMode());
    m_Controller.getOp_Y_FS().whenActive(new IndexerShootingMode());
    m_Controller.getOp_X_FS().whenActive(new FlywheelVelocityToggle());
    m_Controller.getOp_Start_FS().whenActive(new ShootManualOne(60)); //TODO: SET A GOOD DEGREE
    m_Controller.getOp_Back_FS().whenActive(new ShootReset());

    m_Controller.getOp_RightTrigger().whenActive(new ShootTwoBalls());
    m_Controller.getOp_LeftTrigger().whenActive(new ShootOneBall());



    // m_Controller.getOp_A_DS().whenActive(new WinchUp());
    // m_Controller.getOp_B_DS().whenActive(new PrimaryPistonToggle()); ////
    // m_Controller.getOp_X_DS().whenActive(new SecondaryPistonToggle());
    // m_Controller.getOp_Y_DS().whenActive(new WinchDown());
    // m_Controller.getOp_Start_DS().whenActive(new ClimbDisableToggle());
    // m_Controller.getOp_Back_DS().whenActive(new ShootReset());
    // Active Climb
    // m_Controller.getOp_DPAD_UP().whenActive(new ClimbToMid());
    // m_Controller.getOp_DPAD_DOWN().whenActive(new ClimbToTraversal());
    // m_Controller.getOp_DPAD_LEFT().whenActive(new ClimbToHigh());
    // Passive Climb
    // m_Controller.getOp_DPAD_UP().whenActive(new ClimbExtend());
    // m_Controller.getOp_DPAD_DOWN().whenActive(new ClimbRetract());
    // m_Controller.getOp_DPAD_RIGHT().whenActive(new ClimbDisableToggle());

    m_Angler.moveToAngle(m_Controller.getOp_LeftY());

    m_Controller.getOp_A_DS().whenActive(new AnglerPctOutput());
    m_Controller.getOp_B_DS().whenActive(new FeederPctOutput());
    m_Controller.getOp_X_DS().whenActive(new FlywheelPctOutput());
    m_Controller.getOp_Y_DS().whenActive(new I2CTest());
  }

  public void AutonInitRobotCentric() {
    m_CoordSys.setCoordTypeToRobotCentric();
  }
  
  public void TelopInitFieldCentric() {
    m_CoordSys.setCoordTypeToFieldCentric();
  }


  public Command getAutonomousCommand() {
    System.out.println("run");

    String chosen = m_autonChooser.getSelected();
    if(chosen.equals(m_5)) return new Auton5Balls(); //TODO: Put in order of frequency so the bot doesn't have to process more (shouldn't effect anything but just good to have)
    if(chosen.equals(m_4)) return new Auton4Balls();
    if(chosen.equals(m_3)) return new Auton3Balls();
    if(chosen.equals(m_2)) return new Auton2Balls();
    if(chosen.equals(m_MS)) return new Auton1Ball();
    if(chosen.equals(m_S)) return new AutonInstantShoot();
    if(chosen.equals(m_M)) return new AutonExitTarmac();
    if(chosen.equals(m_B)) return new AutonBlock();
    if(chosen.equals(m_6L)) return new AutonMove(6, Math.PI/2);
    if(chosen.equals(m_6R)) return new AutonMove(6, (3 * Math.PI)/2);
    if(chosen.equals(m_6F)) return new AutonMove(6, 0);
    if(chosen.equals(m_6B)) return new AutonMove(6, Math.PI);

    return new AutonExitTarmac(); //TODO: Return the most common auton
  }
}