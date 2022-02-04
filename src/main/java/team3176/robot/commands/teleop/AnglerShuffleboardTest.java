// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.shooter.Angler;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AnglerShuffleboardTest extends CommandBase {
  /** Creates a new AnglerShuffleboardTest. */

  private Angler m_angler = Angler.getInstance();

  public AnglerShuffleboardTest() {
    addRequirements(m_angler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putNumber("Percent Output", 0.0);
    System.out.println("COMMAND INIT");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (SmartDashboard.getNumber("Percent Output", 0.0) <= 0.3) {
      m_angler.engageRawMotor(SmartDashboard.getNumber("Percent Output", 0.0));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
