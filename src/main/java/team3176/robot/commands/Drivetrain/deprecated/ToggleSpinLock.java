package team3176.robot.commands.Drivetrain.deprecated;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Gyro3176;

public class ToggleSpinLock extends InstantCommand {
  private Gyro3176 m_Gyro3176 = Gyro3176.getInstance();
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();


  public ToggleSpinLock() {
    addRequirements(m_Gyro3176);
  }

  @Override
  public void initialize() {
    m_Gyro3176.toggleSpinLock();
  }
}