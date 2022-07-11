// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ClimbConstants;
import team3176.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class Climb extends SubsystemBase {
  private static Climb instance;
  public static Climb getInstance() {
    if(instance == null) {instance = new Climb(new ClimbIO() {});}
    return instance;
  }

  private final ClimbIO io;
  private final ClimbIOInputs inputs = new ClimbIOInputs();


  private DoubleSolenoid climbPistons;
  private boolean isClimbExtened;
  private boolean isSmartDashboardTestControlsShown;
  public String mode = ""; //auto, teleop, test, disabled

  private Climb(ClimbIO io) {
    this.io = io;
    isSmartDashboardTestControlsShown = false;
    climbPistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.PISTON_OPEN_ID, ClimbConstants.PISTON_CLOSE_ID);
    isClimbExtened = false;    
  }


  /* Extends the Pistons */

  public void climbPistonsEngage() {
    climbPistons.set(Value.kForward);
    isClimbExtened = true;
  }

  /* Retracts the Pistons */

  public void climbPistonsRetract() {
    climbPistons.set(Value.kReverse);
    isClimbExtened = false;
  }

  /**Returns the State of the Pistons*/

  public boolean getIsClimbExtended() {return isClimbExtened;}

  /* Puts Tiles on Shuffleboard (by SmartDashboard) to use in TestPeriodic */

  public void putSmartDashboardControlCommands() {
    SmartDashboard.putBoolean("ClimbExtended", isClimbExtened);
  }

  /* Updates Climb by Shuffleboard */

  public void setValuesFromSmartDashboard() {
    boolean lastState = isClimbExtened;
    boolean requestedState = SmartDashboard.getBoolean("ClimbExtended", false);
    if(lastState != requestedState) {
      if(requestedState) {climbPistonsEngage();}
      else {climbPistonsRetract();}
    }
  }

  @Override
  public void periodic() {
    if(mode.equals("test")) {
      if(!isSmartDashboardTestControlsShown) putSmartDashboardControlCommands();
      setValuesFromSmartDashboard();
    }

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Climb", inputs);
    Logger.getInstance().recordOutput("Climb/Extended", isClimbExtened);
  }

  @Override
  public void simulationPeriodic() {}
}