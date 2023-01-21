package team3176.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.subsystems.drivetrain.SwervePod2022;
import team3176.robot.subsystems.drivetrain.Odometry3176;
import team3176.robot.subsystems.drivetrain.Gyro3176;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveSubsystem extends SubsystemBase {

    private static SwerveSubsystem instance;

    private SwervePod2022 m_frontRight;
    private SwervePod2022 m_frontLeft;
    private SwervePod2022 m_backLeft ;
    private SwervePod2022 m_backRight;

    private Odometry3176 m_Odometry = Odometry3176.getInstance();
    private Gyro3176 m_Gyro = Gyro3176.getInstance();

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = 
        new SwerveDriveOdometry( 
            DrivetrainConstants.DRIVE_KINEMATICS, 
            new Rotation2d(m_Gyro.getCurrentChassisYaw()),
            new SwerveModulePosition[] {
                m_frontRight.getPosition(),
                m_frontLeft.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );


    public TalonFX[] driveControllers = { new TalonFX(DrivetrainConstants.THRUST_FR_CID),
       new TalonFX(DrivetrainConstants.THRUST_FL_CID), new TalonFX(DrivetrainConstants.THRUST_BL_CID),
       new TalonFX(DrivetrainConstants.THRUST_BR_CID) };
          
    public CANSparkMax[] azimuthControllers = { new CANSparkMax(DrivetrainConstants.STEER_FR_CID, MotorType.kBrushless),
       new CANSparkMax(DrivetrainConstants.STEER_FL_CID, MotorType.kBrushless), new CANSparkMax(DrivetrainConstants.STEER_BL_CID, MotorType.kBrushless),
       new CANSparkMax(DrivetrainConstants.STEER_BR_CID, MotorType.kBrushless) };
        

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    
       m_frontLeft = new SwervePod2022(0, driveControllers[0], azimuthControllers[0]);
       m_frontRight = new SwervePod2022(1, driveControllers[1], azimuthControllers[1]);
       m_backLeft = new SwervePod2022(2, driveControllers[2], azimuthControllers[2]);
       m_backRight = new SwervePod2022(3, driveControllers[3], azimuthControllers[3]);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        //***odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        //***odometer.update(getRotation2d(),m_frontLeft.getState(), m_frontRight.getState(),m_backLeft.getState(), m_backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
       m_frontLeft.stop();
       m_frontRight.stop();
       m_backLeft.stop();
       m_backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
       m_frontLeft.setDesiredState(desiredStates[0]);
       m_frontRight.setDesiredState(desiredStates[1]);
       m_backLeft.setDesiredState(desiredStates[2]);
       m_backRight.setDesiredState(desiredStates[3]);
    }

    public static SwerveSubsystem getInstance() {
       // if(instance == null) {instance = new SwerveSubsystem(new DrivetrainIO() {});}
        if(instance == null) {instance = new SwerveSubsystem();}
        return instance;
      }
    
}
