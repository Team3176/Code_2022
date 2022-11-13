package team3176.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.clarke.Clarke;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.vision.Vision;

public class ClarkeSpinCorrectionOn extends InstantCommand {
  private Vision m_Vision = Vision.getInstance();
  private Clarke m_Clarke = Clarke.getInstance();
    private Drivetrain m_Drivetrain = Drivetrain.getInstance();


  public ClarkeSpinCorrectionOn() {
    addRequirements(m_Vision);
  }

  @Override
  public void initialize() {
    if (!m_Vision.getIsVisionSpinCorrectionOn()){
      m_Clarke.setClarkeSpinCorrection(true);
    }
  }
}