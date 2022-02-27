package team3176.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;;

public class SwitchCoordType extends InstantCommand {
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  private CoordSys m_coordSys = CoordSys.getInstance();

  public SwitchCoordType() {
    addRequirements(m_Drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("EXECUTING SWITCHCOORDTYPE");
    System.out.println(m_coordSys.getCurrentCoordType());
    if ( m_coordSys.getCurrentCoordType() == coordType.FIELD_CENTRIC ) {
      m_coordSys.setCoordTypeToRobotCentric();;
      System.out.println("SwitchCoordType: ROBOT CENTRIC ACTIVATED");
    } else if ( m_coordSys.getCurrentCoordType() == coordType.ROBOT_CENTRIC ) {
      m_coordSys.setCoordTypeToFieldCentric();;
      System.out.println("SwitchCoordType: FIELD CENTRIC ACTIVATED");

    }
  }
}
