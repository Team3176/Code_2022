package team3176.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.Vision;

public class VisionSpinCorrectionOn extends InstantCommand {
  private Vision m_Vision = Vision.getInstance();
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();


  public VisionSpinCorrectionOn() {
    addRequirements(m_Vision);
  }

  @Override
  public void initialize() {
    m_Vision.setVisionSpinCorrectionOn();
  }
}