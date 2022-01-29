package team3176.robot.commands.teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;

public class SwerveTurboOff extends InstantCommand {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();


  public SwerveTurboOff() {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    m_Drivetrain.setTurbo(false);
  }

}
