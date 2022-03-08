package team3176.robot.commands.Drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;

public class ToggleCoordSys extends InstantCommand {
  private CoordSys m_CoordSys = CoordSys.getInstance();


  public ToggleCoordSys() {
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    System.out.println("ToggleCoordSys command RUN ########################################################################################################################");
    if (m_CoordSys.isFieldCentric()) {
      m_CoordSys.setCoordTypeToRobotCentric();
      System.out.println("ToogleCordSys evaluated under isFieldCentric ########################################################################################################################");
    } else if (m_CoordSys.isRobotCentric()) {
      m_CoordSys.setCoordTypeToFieldCentric();
      System.out.println("ToogleCordSys evaluated under isRobotCentric ########################################################################################################################");
    }
    System.out.println("ToggleCoordSys has finished ########################################################################################################################");
  }

}
