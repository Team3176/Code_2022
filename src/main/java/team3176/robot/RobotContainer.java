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
import team3176.robot.commands.Auton.*;
import team3176.robot.commands.CMD_Groups.*;
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
  // private static final String m_B = "s_Block";
  private static final String m_M = "s_ExitTarmac";
  private static final String m_6L = "s_Move6inToTheLeft";
  private static final String m_6R = "s_Move6inToTheRight";
  private static final String m_6F = "s_Move6inToTheFront";
  private static final String m_6B = "s_Move6inToTheBack";
  private static final String m_9F = "s_Move9inToTheFront";
  private static final String m_9B = "s_Move9inToTheBack";
  private static final String m_TS = "s_ShootAndLeave";
  private static final String m_SI = "s_LeaveAndShootTwo";
  private static final String m_2H = "s_2BallHanger";
  private static final String m_2M = "s_2BallMid";
  private static final String m_MS = "s_MoveAndShoot";
  private static final String m_3B = "s_3Ball";
  private static final String m_3BS = "s_3BallSlow";
  private static final String m_4B = "s_4Ball";
  private static final String m_5B = "s_5Ball";
  private static final String m_3H = "s_3BallHanger";
  private static final String m_2C = "s_2BallCitrus";
  private static final String m_2EC = "s_2BallExtraCitrus";
  private static final String m_Int = "s_Interfere";
  private static final String m_Rot = "s_Rot";
  private static final String m_TrapRot = "s_TrapRot";
  private static final String m_TrapDriveRot = "s_TrapDriveRot";

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
    // TODO: ADD A WAY TO CLEAR STICKY FAULTS
    // m_Compressor.disable(); //HAVE TO TELL IT TO DISABLE FOR IT TO NOT AUTO START
    m_Compressor.enableDigital();

    m_Flywheel.setDefaultCommand(new FlywheelDefaultCommand(0.31, 0.2));

    if (!MasterConstants.IS_TUNING_MODE) {
      m_Drivetrain.setDefaultCommand(new SwerveDrive(
          () -> m_Controller.getForward(),
          () -> m_Controller.getStrafe(),
          () -> m_Controller.getSpin()// ,
      // () -> m_Controller.isFieldCentricButtonPressed(),
      // () -> m_Controller.isRobotCentricButtonPressed()
      ));
    } else {
      m_Drivetrain.setDefaultCommand(new SwerveDriveTune());
    }

    m_autonChooser = new SendableChooser<>();
    m_autonChooser.setDefaultOption("Auto: ExitTarmac", m_M);
    // m_autonChooser.addOption("Auto: Block", m_B);
    m_autonChooser.addOption("Auto: Move 6in Left", m_6L);
    m_autonChooser.addOption("Auto: Move 6in Right", m_6R);
    m_autonChooser.addOption("Auto: Move 6in Forward", m_6F);
    m_autonChooser.addOption("Auto: Move 6in Backwards", m_6B);
    m_autonChooser.addOption("Auto: Move 9in Forward", m_9F);
    m_autonChooser.addOption("Auto: Move 9in Backwards", m_9B);
    m_autonChooser.addOption("Auto: Shoot and Exit Tarmac", m_TS);
    m_autonChooser.addOption("Auto: 2 Ball (Right)", m_SI);
    m_autonChooser.addOption("Auto: 2 Ball (Left/Hanger)", m_2H);
    m_autonChooser.addOption("Auto: 2 Ball (Middle)", m_2M);
    m_autonChooser.addOption("Auto: Exit and Shoot", m_MS);
    m_autonChooser.addOption("Auto: 3 Ball (Right)", m_3B);
    m_autonChooser.addOption("Auto: 3 Ball Slow (Right)", m_3BS);
    m_autonChooser.addOption("Auto: 3 Ball (Left/Hanger)", m_3H);
    m_autonChooser.addOption("Auto: 4 Ball", m_4B);
    m_autonChooser.addOption("Auto: 5 Ball", m_5B);
    m_autonChooser.addOption("Auto: 2 Ball Citrus (Left/Hanger)", m_2C);
    m_autonChooser.addOption("Auto: 2 Ball Extra Citrus (Left/Hanger)", m_2EC);
    m_autonChooser.addOption("Auto: Interfere (Left/Hanger)", m_Int);
    m_autonChooser.addOption("Auto: Rotation", m_Rot);
    m_autonChooser.addOption("Auto: TrapRotate", m_TrapRot);
    m_autonChooser.addOption("Auto: TrapDriveRotate", m_TrapDriveRot);
    SmartDashboard.putData("Auton Choice", m_autonChooser);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_Controller.getTransStick_Button1().whenHeld(new SwerveTurboOn());
    m_Controller.getTransStick_Button1().whenReleased(new SwerveTurboOff());
    m_Controller.getTransStick_Button3().whenHeld(new SwerveDefense());
    //m_Controller.getTransStick_Button4().whenPressed(new ToggleCoordSys());
    m_Controller.getTransStick_Button4().whenHeld(new CoordTypeToRobotCentric());
    m_Controller.getTransStick_Button4().whenReleased(new CoordTypeToFieldCentric());

    m_Controller.getTransStick_HAT_0().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        0.0));
    m_Controller.getTransStick_HAT_45().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        45.0));
    m_Controller.getTransStick_HAT_135().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        135.0));
    m_Controller.getTransStick_HAT_180().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        180.0));
    m_Controller.getTransStick_HAT_225().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        225.0));
    m_Controller.getTransStick_HAT_315().whileHeld(new SwervePivotAtPodBi(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin(), 
        315.0));

    m_Controller.getRotStick_Button1().whileActiveOnce(new FlywheelAngleVisionIntAutoFire());
    m_Controller.getRotStick_Button1().whenHeld(new VisionSpinCorrectionOn());
    m_Controller.getRotStick_Button1().whenReleased(new VisionSpinCorrectionOff());
    m_Controller.getRotStick_Button2().whileActiveOnce(new ShootVisionAutoFire());
    /*
    m_Controller.getRotStick_Button2().whileActiveOnce(new ShootVisionTrueAutoFire(
        () -> m_Controller.getForward(),
        () -> m_Controller.getStrafe(),
        () -> m_Controller.getSpin() 
    ));
    */
    // m_Controller.getRotStick_Button3().whenPressed(new ToggleSpinLock());
    m_Controller.getRotStick_Button3().whenHeld(new ClarkeSpinCorrectionOn());
    m_Controller.getRotStick_Button3().whenHeld(new ClarkeSpinCorrectionOff());
    //m_Controller.getRotStick_Button3().whenReleased(new SwerveSpinLockOff());
    //m_Controller.getRotStick_Button3().whenReleased(new SwerveSpinLockOff());
    m_Controller.getRotStick_Button4().whenPressed(new SwerveResetGyro());
    // m_Controller.getRotStick_Button5().whenPressed(new SwervePodsAzimuthGoHome());

    m_Controller.getOp_A().whileActiveOnce(new IntakingDirect2());
    m_Controller.getOp_A().whenInactive(new DelayedIntakeStop());

    m_Controller.getOp_Y().whileActiveOnce(new ShootSetVals());
    m_Controller.getOp_B().whenActive(new FlywheelStop());

    m_Controller.getOp_Back_FS().whileActiveOnce(new IndexerBackWhenHeld());
    m_Controller.getOp_Start_FS().whileActiveOnce(new IndexerForwardWhenHeld());
    m_Controller.getOp_Back_DS().whenActive(new ExtendIntake());
    m_Controller.getOp_Start_DS().whenActive(new RetractIntake());

    m_Controller.getOp_A_DS().whenActive(new ClimbPistonEngage()); //TODO: CHECK IF TWO COMMANDS CAN BE MAPPED TO THE SAME BUTTON
    m_Controller.getOp_A_DS().whenActive(new AnglerZeroAtMax());
    m_Controller.getOp_B_DS().whenActive(new ClimbPistonRetract());
    
    m_Controller.getOp_DPAD_UP().whenActive(new VisionDriverCam());
    m_Controller.getOp_DPAD_DOWN().whenActive(new VisionZoom2x());

    m_Controller.getOp_DPAD_LEFT().whenActive(new FlywheelAngleFender());
    m_Controller.getOp_DPAD_RIGHT().whenActive(new FlywheelAngleWall());

    m_Controller.getOp_A_FS().whileActiveOnce(new AnglerZeroAtMax());
    m_Controller.getOp_Y_FS().whenActive(new FlywheelDefaultCommandStop());

    m_Controller.getOp_Back().whileActiveOnce(new SpittingDown());
    m_Controller.getOp_Start().whileActiveOnce(new SpittingUp());

    // m_Controller.getOp_Back().whileActiveOnce(new FlywheelPIDToggleTest());
    // m_Controller.getOp_Start().whileActiveOnce(new ShootPIDToggleTest());

    m_Controller.getOp_X().whileActiveOnce(new FlywheelAngleVision());

    m_Controller.getOp_LeftTrigger().whileActiveOnce(new ShootVision());

    m_Controller.getOp_RightTrigger().whileActiveOnce(new FlywheelAngleVisionInt());
  }

  public void AutonInitRobotCentric() {
    m_CoordSys.setCoordTypeToRobotCentric();
  }

  public void TelopInitFieldCentric() {
    m_CoordSys.setCoordTypeToFieldCentric();
  }

  public Command getAutonomousCommand() {
    String chosen = m_autonChooser.getSelected();

    if (chosen.equals(m_M))
      return new AutonExitTarmac();
    // if(chosen.equals(m_B)) return new AutonBlock();
    if (chosen.equals(m_6L))
      return new TrapezoidDrive(0, -6);
    if (chosen.equals(m_6R))
      return new TrapezoidDrive(0, 6);
    if (chosen.equals(m_6F))
      return new TrapezoidDrive(6, 0);
    if (chosen.equals(m_6B))
      return new TrapezoidDrive(-6, 0);
    if (chosen.equals(m_9F))
      return new TrapezoidDrive(9, 0);
    if (chosen.equals(m_9B))
      return new TrapezoidDrive(-9, 0);
    if (chosen.equals(m_TS))
      return new AutoInTarmacShoot();
    if (chosen.equals(m_SI))
      return new Auto2Balls();
    if (chosen.equals(m_2H))
      return new Auto2BallsAtHanger();
    if (chosen.equals(m_2M))
      return new Auto2BallsMiddle();
    if (chosen.equals(m_MS))
      return new AutoMoveAndShoot();
    if (chosen.equals(m_3B))
      return new Auto3Balls();
    if (chosen.equals(m_3BS))
      return new Auto3BallSlow();
    if (chosen.equals(m_4B))
      return new Auto4Ball();
    if (chosen.equals(m_5B))
      return new Auto5Ball();
    if (chosen.equals(m_3H))
      return new Auton3BallAtHanger();
    if (chosen.equals(m_2C))
      return new Auto2BallCitrus();
    if (chosen.equals(m_2EC))
      return new Auto2BallExtraCitrus();
    if (chosen.equals(m_Int))
      return new AutoInterfere();
    if (chosen.equals(m_Rot))
      return new AutonRotate(0.15, 90);
    if (chosen.equals(m_TrapRot))
      return new TrapezoidRotate(-1, 20);
    if (chosen.equals(m_TrapDriveRot))
      return new TrapezoidDriveRotate(3,0, 1, 5); 


    return new AutoInTarmacShoot();
  }
}
