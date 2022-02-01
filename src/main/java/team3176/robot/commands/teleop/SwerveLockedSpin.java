package team3176.robot.commands.teleop;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Drivetrain.driveMode;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import team3176.robot.util.PIDLoop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveLockedSpin extends InstantCommand {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Gyro3176 m_gyro = Gyro3176.getInstance();
  public SwerveLockedSpin() {
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    
    m_gyro.setSpinLockAngle(); //Rearragned because the error would be big currAngle - 0 as error
    drivetrain.toggleSpinLock();
  }

}
