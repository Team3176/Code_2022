package team3176.robot.commands.Drivetrain.imported;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;

public class CoordTypeToFieldCentric extends InstantCommand {
  private CoordSys m_CoordSys = CoordSys.getInstance();


  public CoordTypeToFieldCentric() {
    addRequirements(m_CoordSys);
  }

  @Override
  public void initialize() {
    m_CoordSys.setCoordType(coordType.FIELD_CENTRIC);
  }

}
