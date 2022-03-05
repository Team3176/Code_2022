// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.constants.AnglerConstants;
import team3176.robot.subsystems.Angler;

/**
 * Moves the Angler until it reaches the lower limit switch. When it is reached, the Angler's encoder position is set to zero. Running this
 * (or the AnglerZeroAtMax command) on robot start is ABSOLUTELY NECESSARY if we want to set the Angler to a position based on a given
 * location instead of simply a change, as it will need a reference point to do that. This command provides that reference point until the
 * robot is power cycled.
 * @author Jared Brown
 */
public class AnglerZeroAtMax extends CommandBase {
  /** Creates a new AnglerZeroAtMax. */
  private Angler m_angler = Angler.getInstance();

  public AnglerZeroAtMax() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_angler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angler.setVelocity(AnglerConstants.kDegreesPerSecondForZeroing);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.zeroAtMax();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_angler.getLimiterHigh();
  }
}
