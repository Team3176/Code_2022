package team3176.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys.coordType;
import team3176.robot.subsystems.vision.Vision;

public class VisionSpinCorrectionOff extends InstantCommand {
  private Vision m_Vision = Vision.getInstance();
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();


  public VisionSpinCorrectionOff() {
    addRequirements(m_Vision);
  }

  @Override
  public void initialize() {
    m_Vision.setVisionSpinCorrection(false);
  }
}