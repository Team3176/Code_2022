package team3176.robot.subsystems.SwerveSubsystem;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.SwerveSubsystemConstants;
import team3176.robot.subsystems.SwerveSubsystem.SwervePod2022;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveSubsystem extends SubsystemBase {
    private final SwervePod2022 frontLeft;

    private final SwervePod2022 frontRight;

    private final SwervePod2022 backLeft ;

    private final SwervePod2022 backRight;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(SwerveSubsystemConstants.DRIVE_KINEMATICS,
            new Rotation2d(0));

    public TalonFX[] driveControllers = { new TalonFX(SwerveSubsystemConstants.THRUST_FR_CID),
       new TalonFX(SwerveSubsystemConstants.THRUST_FL_CID), new TalonFX(SwerveSubsystemConstants.THRUST_BL_CID),
       new TalonFX(SwerveSubsystemConstants.THRUST_BR_CID) };
          
    public CANSparkMax[] azimuthControllers = { new CANSparkMax(SwerveSubsystemConstants.STEER_FR_CID, MotorType.kBrushless),
       new CANSparkMax(SwerveSubsystemConstants.STEER_FL_CID, MotorType.kBrushless), new CANSparkMax(SwerveSubsystemConstants.STEER_BL_CID, MotorType.kBrushless),
       new CANSparkMax(SwerveSubsystemConstants.STEER_BR_CID, MotorType.kBrushless) };
        

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    
        frontLeft = new SwervePod2022(0, driveControllers[0], azimuthControllers[0]);
        frontRight = new SwervePod2022(1, driveControllers[1], azimuthControllers[1]);
        backLeft = new SwervePod2022(2, driveControllers[2], azimuthControllers[2]);
        backRight = new SwervePod2022(3, driveControllers[3], azimuthControllers[3]);
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
        odometer.resetPosition(pose, getRotation2d());
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                backRight.getState());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSubsystemConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
