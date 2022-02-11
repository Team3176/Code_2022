// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.lib.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.climb.ClimbActive;

/**
 * Uses a passed joystick value to Manually control the Winch
 */

public class ClimbWinchManual extends CommandBase {
  private ClimbActive m_Climb = ClimbActive.getInstance();
  private boolean direction; //true is Up and false is Down

  public ClimbWinchManual(double joyValue) { //TODO: SEE IF IT IS BETTER WITH PASSING THE JOYVAL AS A PARAMETER OR IN COMMAND WITH ACCESSING THE INSTANCE OF CONTROLLER BUT CANT DO ON THIS BRANCH
    addRequirements(m_Climb);
    if(joyValue > 0) {direction = true;} else {direction = false;}
  }

  @Override
  public void initialize() {
    if(direction) {m_Climb.winchManualUp();}
    else {m_Climb.winchManualDown();}
  }

  @Override
  public void execute() {
    if(direction) {m_Climb.winchManualUp();}
    else {m_Climb.winchManualDown();}
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
