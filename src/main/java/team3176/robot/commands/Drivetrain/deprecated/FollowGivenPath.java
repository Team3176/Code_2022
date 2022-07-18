// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3176.robot.commands.Drivetrain.deprecated;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import team3176.robot.Robot;
import team3176.robot.RobotContainer;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.Odometry3176;

public class FollowGivenPath extends CommandBase {
  
  private Drivetrain drivetrain;
  private Trajectory trajectory;
  private Robot robot;
  private RobotContainer container;
  private Odometry3176 odometry;
  
  /** Creates a new FollowSlalomPath. */
  public FollowGivenPath(Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrain = Drivetrain.getInstance();
    odometry = Odometry3176.getInstance();
    
   

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    /* container.swerveControllerCommand =
    new SwerveControllerCommand(
        trajectory,
        drivetrain::getCurrentPose, 
        DrivetrainConstants.DRIVE_KINEMATICS,

        // Position controllers
        new PIDController(DrivetrainConstants.P_X_Controller, 0, 0),
        new PIDController(DrivetrainConstants.P_Y_Controller, 0, 0),
        container.thetaController,
        drivetrain::setModuleStates, //Not sure about setModuleStates
        drivetrain);*/

// Reset odometry to the starting pose of the trajectory.
odometry.resetOdometry(trajectory.getInitialPose());


}
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
