package team3176.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.SwerveSubsystem;
import team3176.robot.subsystems.clarke.Clarke;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;

public class ClarkeSpinCorrectionOff extends InstantCommand {
  private Clarke m_Clarke = Clarke.getInstance();
  private SwerveSubsystem m_SwerveSubsystem = SwerveSubsystem.getInstance();


  public ClarkeSpinCorrectionOff() {
    addRequirements(m_Clarke);
  }

  @Override
  public void initialize() {
    m_Clarke.setClarkeSpinCorrection(false);
  }
}