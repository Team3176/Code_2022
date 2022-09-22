package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;

import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.Odometry3176;
import team3176.robot.subsystems.SwerveSubsystem.CoordSys.coordType;

public class HolonomicAuton extends CommandBase {

  SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();
  Odometry3176 odometry = Odometry3176.getInstance();

  HolonomicDriveController holonomicController;
  Trajectory trajectory;

  double startTime;
  
  int stateNumber;
  
  public HolonomicAuton(Trajectory trajectory) {
    holonomicController = new HolonomicDriveController(
      new PIDController(1, 0, 0), // X Controller
      new PIDController(1, 0, 0), // Y Controller
      new ProfiledPIDController(1, 0, 0, // Theta Controller
        new TrapezoidProfile.Constraints(SwerveSubsystemConstants.MAX_ROT_SPEED_RADIANS_PER_SECOND, 2 * Math.PI))); // Constraints are maxVel and maxAccel both in radians

    this.trajectory = trajectory;
    stateNumber = 0;
  }

  @Override
  public void initialize() {
      startTime = Timer.getFPGATimestamp();
  }

  
  @Override
  public void execute() {
    double trajTime = 0.0;
    for(int idx = stateNumber; idx < trajectory.getStates().size(); idx++) {
      if(trajectory.getStates().get(idx).timeSeconds < Timer.getFPGATimestamp() - startTime) {
        trajTime = trajectory.getStates().get(idx - 1).timeSeconds;
        stateNumber = idx - 1;
        break;
      }
    }
       
    Trajectory.State nextState = trajectory.sample(trajTime);
    ChassisSpeeds adjustedSpeeds = holonomicController.calculate(odometry.getCurrentPose(), nextState, nextState.poseMeters.getRotation());

    // Simpily normalizing to get -1 to 1
    double forwardCommand = adjustedSpeeds.vxMetersPerSecond / SwerveSubsystemConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND;
    double strafeCommand = adjustedSpeeds.vyMetersPerSecond / SwerveSubsystemConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND;
    double spinCommand = adjustedSpeeds.omegaRadiansPerSecond / SwerveSubsystemConstants.MAX_ROT_SPEED_RADIANS_PER_SECOND;

    SwerveSubsystem.drive(forwardCommand, strafeCommand, spinCommand, coordType.FIELD_CENTRIC);
  }

  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < Timer.getFPGATimestamp() - startTime;
  } 
}