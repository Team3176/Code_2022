// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.CMD_Groups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.ShooterLocationValues;
import team3176.robot.subsystems.angler.Angler;
import team3176.robot.subsystems.flywheel.Flywheel;

public class FlywheelAngleFender extends CommandBase {
  Flywheel m_Flywheel = Flywheel.getInstance();
  Angler m_Angler = Angler.getInstance();
  public FlywheelAngleFender() {
    addRequirements(m_Flywheel, m_Angler);
  }

  @Override
  public void initialize() {
    m_Angler.moveToAngle(ShooterLocationValues.POINTS[0][5]);
    //m_Flywheel.spinMotorsVelocityPID(ShooterLocationValues.POINTS[0][3], ShooterLocationValues.POINTS[0][4]);
    m_Flywheel.setPCT(.3,.3);
  }

  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {
    m_Flywheel.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
