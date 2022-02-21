package team3176.robot.subsystems.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*; 
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import team3176.robot.RobotContainer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import team3176.robot.constants.DrivetrainConstants;
import team3176.robot.constants.SwervePodConstants2022;
import team3176.robot.constants.MasterConstants;
import com.ctre.phoenix.sensors.CANCoder;


public class SwervePod2022Backup {

    private TalonFX driveController;
    private CANSparkMax azimuthController;
    private SparkMaxPIDController azimuthPIDController;
    CANCoder azimuthEncoder;
    double azimuthEncoderPosition;

    private int id;
    private double kEncoderOffset; 
    private double kazimuthEncoderUnitsPerRevolution;
    private double kDriveEncoderUnitsPerRevolution;
    private int off = 0;

    private double lastEncoderPos;
    private double radianError;
    private double radianPos;
    private double encoderError;
    private double encoderPos;

    private double driveCommand;
    private double velTicsPer100ms;

    public int kSlotIdx_azimuth, kPIDLoopIdx_azimuth, kTimeoutMs_azimuth,kSlotIdx_drive, kPIDLoopIdx_drive, kTimeoutMs_drive;

    private double podDrive, podazimuth;

    private double kP_azimuth;
    private double kI_azimuth;
    private double kD_azimuth;
    private double kFF_azimuth;
    private double kIz_azimuth;
    private double kMaxOutput;
    private double kMinOutput;


    private double kP_Drive;
    private double kI_Drive;
    private double kD_Drive;
    private double kF_Drive;

    private double maxVelTicsPer100ms;
    private boolean isAutonSwerveControllerOptimizingazimuthPos = false;

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants2022.CHASSIS_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND;

    private double startTics;

    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turningPIDController;
    private SwerveModuleState state;
    private RelativeEncoder m_encoder;

    public SwervePod2022Backup(int id, TalonFX driveController, CANSparkMax azimuthController) {
        this.id = id;
        azimuthEncoder = new CANCoder(SwervePodConstants2022.STEER_CANCODER_CID[id]);

        this.kEncoderOffset = SwervePodConstants2022.AZIMUTH_OFFSET[this.id];
        ///System.out.println("P"+(this.id+1)+" kEncoderOffset: "+this.kEncoderOffset);

        kazimuthEncoderUnitsPerRevolution = SwervePodConstants2022.AZIMUTH_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_azimuth = SwervePodConstants2022.TALON_AZIMUTH_PID_SLOT_ID;
        kPIDLoopIdx_azimuth = SwervePodConstants2022.TALON_AZIMUTH_PID_LOOP_ID;
        kTimeoutMs_azimuth = SwervePodConstants2022.TALON_AZIMUTH_PID_TIMEOUT_MS;

        m_drivePIDController = new PIDController(SwervePodConstants2022.P_MODULE_THRUST_CONTROLLER, 0, 0);

        m_turningPIDController = new ProfiledPIDController(
            SwervePodConstants2022.P_MODULE_TURNING_CONTROLLER[0], 0, SwervePodConstants2022.P_MODULE_TURNING_CONTROLLER[2],
            new TrapezoidProfile.Constraints(
                (SwervePodConstants2022.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND),
                (SwervePodConstants2022.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND)));
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_turningPIDController.reset(0.0, 0.0);
        

        /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
        //azimuthController.configAllowableClosedloopError(0, SwervePodConstants.kPIDLoopIdx, SwervePodConstants.kTimeoutMs);

        kDriveEncoderUnitsPerRevolution = SwervePodConstants2022.THRUST_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_drive = SwervePodConstants2022.TALON_THRUST_PID_SLOT_ID[this.id];
        kPIDLoopIdx_drive = SwervePodConstants2022.TALON_THRUST_PID_LOOP_ID[this.id];
        kTimeoutMs_drive = SwervePodConstants2022.TALON_THRUST_PID_TIMEOUT_MS[this.id];

        m_encoder = azimuthController.getEncoder();

        kP_azimuth = SwervePodConstants2022.AZIMUTH_PID[0][id];
        kI_azimuth = SwervePodConstants2022.AZIMUTH_PID[1][id];
        kD_azimuth = SwervePodConstants2022.AZIMUTH_PID[2][id];
        kFF_azimuth = SwervePodConstants2022.AZIMUTH_PID[3][id];
        kIz_azimuth = SwervePodConstants2022.AZIMUTH_PID[4][id];
        kMaxOutput = SwervePodConstants2022.AZIMUTH_PID[5][id];
        kMinOutput = SwervePodConstants2022.AZIMUTH_PID[6][id];

        kP_Drive = 0.03; // SwervePodConstants.DRIVE_PID[0][id];
        kI_Drive = 0.0; // SwervePodConstants.DRIVE_PID[1][id];
        kD_Drive = 0.0; // SwervePodConstants.DRIVE_PID[2][id];
        kF_Drive = .045; // SwervePodConstants.DRIVE_PID[3][id];

        this.driveController = driveController;
        this.azimuthController = azimuthController;
        this.azimuthPIDController = azimuthController.getPIDController();

        this.driveController.configFactoryDefault();
        this.azimuthController.restoreFactoryDefaults();

        this.driveController.configClosedloopRamp(0.5);    

       // this.driveController.setNeutralMode(NeutralMode.Brake);
       // this.driveController.setNeutralMode(NeutralMode.Brake);

        this.driveController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        //this.azimuthController.configSelectedFeedbackSensor(FeedbackDevice.m_encoder, 0, 0);   //TODO: investigate QuadEncoder vs CTRE_MagEncoder_Absolute.  Are the two equivalent?  Why QuadEncoder instead of CTRE_MagEncoder_Absolute

        /*
        if (this.id == 0 || this.id == 1) {
            this.azimuthController.setSensorPhase(SwervePodConstants2022.kSensorPhase);
            this.azimuthController.setInverted(SwervePodConstants2022.kMotorInverted);
        }
        if (this.id == 2 || this.id == 3) {
            this.azimuthController.setSensorPhase(true);
            this.azimuthController.setInverted(true);
        }
        */

            //TODO: check out "Feedback Device Not Continuous"  under config tab in CTRE-tuner.  Is the available via API and set-able?  Caps encoder to range[-4096,4096], correct?
                //this.azimuthController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition), 0, 0);
                //this.azimuthController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute), 0, 0);

        this.driveController.config_kP(kPIDLoopIdx_drive, kP_Drive, kTimeoutMs_drive);
        this.driveController.config_kI(kPIDLoopIdx_drive, kI_Drive, kTimeoutMs_drive);
        this.driveController.config_kD(kPIDLoopIdx_drive, kD_Drive, kTimeoutMs_drive);
        this.driveController.config_kF(kPIDLoopIdx_drive, kF_Drive, kTimeoutMs_drive);

      

        // SmartDashboard.putNumber("P", kP_Drive);
        // SmartDashboard.putNumber("I", kI_Drive);
        // SmartDashboard.putNumber("D", kD_Drive);
        // SmartDashboard.putNumber("F", kF_Drive);
       // SmartDashboard.putNumber("driveSet",0);

        this.azimuthPIDController.setP(kP_azimuth);
        this.azimuthPIDController.setI(kI_azimuth);
        this.azimuthPIDController.setD(kD_azimuth);
        this.azimuthPIDController.setFF(kFF_azimuth);
        this.azimuthPIDController.setIZone(kIz_azimuth);
        this.azimuthPIDController.setOutputRange(kMinOutput,kMaxOutput);

        /*
        this.azimuthPIDController.setP(kP_azimuth, kPIDLoopIdx_azimuth);
        this.azimuthPIDController.setI(kI_azimuth, kPIDLoopIdx_azimuth);
        this.azimuthPIDController.setD(kD_azimuth, kPIDLoopIdx_azimuth);
        this.azimuthPIDController.setFF(kFF_azimuth, kPIDLoopIdx_azimuth);
        this.azimuthPIDController.setIZone(kIz_azimuth, kPIDLoopIdx_azimuth);
        this.azimuthPIDController.setOutputRange(kMinOutput, kMaxOutput);
        */

        // SmartDashboard.putNumber("startTics", startTics);

        // SmartDashboard.putBoolean("pod" + (id + 1) + " inversion", isInverted());
    }

    /**
     * @param podDrive represents desired thrust of swervepod Range = -1 to 1 or
     *                 ft-per-sec?
     * @param podazimuth  represents desired angle of swervepod. Range = -pi to pi.
     */
    public void set(double podDrive, double podazimuth) {
        this.podDrive = podDrive;
        this.podazimuth = podazimuth; 
         //this.azimuthController.config_kP(kSlotIdx_azimuth, SmartDashboard.getNumber("P", kP_azimuth), kTimeoutMs_azimuth);
         //this.azimuthController.config_kI(kSlotIdx_azimuth, SmartDashboard.getNumber("I", kI_azimuth), kTimeoutMs_azimuth);
         //this.azimuthController.config_kD(kSlotIdx_azimuth, SmartDashboard.getNumber("D", kD_azimuth), kTimeoutMs_azimuth);
         //this.azimuthController.config_kF(kSlotIdx_azimuth, SmartDashboard.getNumber("F", kF_azimuth), kTimeoutMs_azimuth);
        // this.driveController.config_kP(kSlotIdx_drive, SmartDashboard.getNumber("P", kP_Drive), kTimeoutMs_azimuth);
        // this.driveController.config_kI(kSlotIdx_drive, SmartDashboard.getNumber("I", kI_Drive), kTimeoutMs_azimuth);
        // this.driveController.config_kD(kSlotIdx_drive, SmartDashboard.getNumber("D", kD_Drive), kTimeoutMs_azimuth);
        // this.driveController.config_kF(kSlotIdx_drive, SmartDashboard.getNumber("F", kF_Drive), kTimeoutMs_azimuth);
        
        
        // SmartDashboard.putNumber("P" + (id + 1) + " podDrive", this.podDrive);
        // SmartDashboard.putNumber("P" + (id + 1) + " podazimuth", this.podazimuth);
            // TODO: need check ether output values. speed vs %-values
        // this.maxVelTicsPer100ms = 1 * 987.2503 * kDriveEncoderUnitsPerRevolution / 600.0;
        // this.velTicsPer100ms = this.podDrive * 2000.0 * kDriveEncoderUnitsPerRevolution / 600.0;  //TODO: rework "podDrive * 2000.0"
        this.maxVelTicsPer100ms = fps2ums(DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND);
        this.velTicsPer100ms = fps2ums(this.podDrive);
        // SmartDashboard.putNumber("fps2ums:velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("podDrive", this.podDrive);
       // velTicsPer100ms = SmartDashboard.getNumber("driveSet",velTicsPer100ms);
        double encoderSetPos = calcazimuthPos(this.podazimuth);
        double tics = rads2Tics(this.podazimuth);
        // SmartDashboard.putNumber("P" + (id + 1) + " tics", tics);
        // SmartDashboard.putNumber("P" + (id + 1) + " absTics", azimuthController.getSelectedSensorPosition());
        //if (this.id == 3) {azimuthController.set(ControlMode.Position, 0.0); } else {   // TODO: Try this to force pod4 to jump lastEncoderPos
        if (this.podDrive > (-Math.pow(10,-10)) && this.podDrive < (Math.pow(10,-10))) {      //TODO: convert this to a deadband range.  abs(podDrive) != 0 is notationally sloppy math
            azimuthPIDController.setReference(this.encoderPos, CANSparkMax.ControlType.kPosition);  
            // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);
        } else {
            azimuthPIDController.setReference(this.encoderPos, CANSparkMax.ControlType.kPosition);  
            this.lastEncoderPos = encoderSetPos;
            // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);
        }    
        //SmartDashboard.putNumber("P" + (id) + "getSelSenPos", azimuthController.getSelectedSensorPosition());

        // SmartDashboard.putNumber("P"+this.id+".podDrive", podDrive);
        //SmartDashboard.putNumber("actualVel", driveController.getVoltage());
        
        // if (this.id == 0){
        //     SmartDashboard.putNumber("Pod3Distance", driveController.getSelectedSensorPosition());
        //     SmartDashboard.putNumber("velTicsPer100ms", velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error", driveController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error2", driveController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("DriveController Velocity", driveController.getSelectedSensorVelocity());
        // } 
        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
        // SmartDashboard.putNumber("P" + (id + 1) + " velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderSetPos_end", encoderSetPos);
        //}
    }

    /**
     * @param angle desired angle of swerve pod in units of radians, range from -PI to +PI
     * @return
     */
    private double calcazimuthPos(double angle) {
        // SmartDashboard.putNumber("P" + (id + 1) + " calcazimuthPos_angle", angle);
        //System.out.println("calcazimuthPos - P"+(this.id+1)+" kEncoderOffset: "+this.kEncoderOffset);

        this.encoderPos = m_encoder.getPosition();
        //this.encoderPos = azimuthController.getSelectedSensorPosition() - this.kEncoderOffset;
        // SmartDashboard.putNumber("P" + (id + 1) + " kEncoderOffset", this.kEncoderOffset);
        // SmartDashboard.putNumber("P" + (id + 1) + " getSelectedSensorPosition", azimuthController.getSelectedSensorPosition());
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderPos_in_calcazimuthPos",this.encoderPos);
        radianPos = tics2Rads(this.encoderPos);
        // SmartDashboard.putNumber("P" + (id + 1) + " radianPos", radianPos);
        radianError = angle - radianPos;
        // SmartDashboard.putNumber("P" + (id + 1) + " radianError", radianError);
        // FYI: Math.copySign(magnitudeVar, signVar) = magnitude value with same sign as signvar

        //if (Math.abs(radianError) > (5 * (PI / 2))) {
        //    System.out.println("Error: Overload");
        //} else if (Math.abs(radianError) > (3 * (PI / 2))) {

        if (isAutonSwerveControllerOptimizingazimuthPos == false) { 
            if (Math.abs(radianError) > (3 * (PI / 2))) {      // TODO: See if commenting out "Thrust-vector sign-flip" fixes
                radianError -= Math.copySign(2 * PI, radianError);
            } else if (Math.abs(radianError) > (PI / 2)) {
                radianError -= Math.copySign(PI, radianError);
                this.velTicsPer100ms = -this.velTicsPer100ms;
            }
        }
        encoderError = rads2Tics(radianError);
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderError", encoderError);
        driveCommand = encoderError + this.encoderPos + this.kEncoderOffset;
        // SmartDashboard.putNumber("P" + (id + 1) + "tics2radianDrivecommand", driveCommand);
        return (driveCommand);
    }

    public void goHome() {
        double homePos = 0 + this.kEncoderOffset;
        this.azimuthPIDController.setReference(homePos, CANSparkMax.ControlType.kPosition);
    }

    private int rads2Tics(double rads) {        //TODO: put a modulo cap limit like in tics2Rads (range[-pi,pi])  (Is it returning 0-2pi somehow?)
        //rads = rads * (2 * Math.PI);
        double rads_clamped = (Math.min((Math.max(rads,(-Math.PI))), (Math.PI)));        
        double tics = ((rads_clamped / (2.0*Math.PI)) * kazimuthEncoderUnitsPerRevolution);
        return (int) tics;
    }

    private double tics2Rads(double tics) {
        tics = tics % kazimuthEncoderUnitsPerRevolution;
        if(tics < 0) {
            tics += kazimuthEncoderUnitsPerRevolution;
        }
        tics -= (kazimuthEncoderUnitsPerRevolution / 2);
        return ((tics / kazimuthEncoderUnitsPerRevolution) * (2 * PI));
    }

    /**
     * @param i feet per second
     * @return tics per 100ms
     */
    private double fps2ums(double i) {
        
        // input * inchesPerFoot * circumfrenceOfWheel * ticsPerRev * gearRatio * secTo100ms
        return i * 12.0 * (1.0/10.21) * 2048.0 *6.17 * .1;
        
        // return i * 100;
    }

    public boolean isInverted() { return azimuthController.getInverted(); }
    public void setInverted() { azimuthController.setInverted(!isInverted()); }

    public void setDesiredState(SwerveModuleState desiredState) {
        //this.isAutonSwerveControllerOptimizingazimuthPos = true;
        // Optimize the reference state to avoid azimuthning further than 90 degrees
        SwerveModuleState newDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle.times(-1));
        Rotation2d rotation = new Rotation2d(-tics2Rads(m_encoder.getPosition()));
        state = newDesiredState;
        //SwerveModuleState.optimize(desiredState, rotation); //I do not know if this is the angle of the encoder.
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_desiredDegrees", desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("P"+this.id+".setdesiredState_sensoredDegrees", rotation.getDegrees());
        double driveOutput =    
            m_drivePIDController.calculate(getVelocity_metersPerSec(), state.speedMetersPerSecond);

        driveOutput = .25 * driveOutput/DrivetrainConstants.MAX_WHEEL_SPEED_METERS_PER_SECOND;   //TODO: <-- Ask Chase, why we multiply by 0.25?

        final var turnOutput = 
            m_turningPIDController.calculate(tics2Rads(m_encoder.getPosition()), state.angle.getRadians());
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_TurnOutput",turnOutput);
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_DriveOutput",driveOutput);

        Rotation2d tempTurnOutput = new Rotation2d(turnOutput);

        SwerveModuleState calculatedState = new SwerveModuleState(driveOutput, tempTurnOutput);
        //if (this.isAutonSwerveControllerOptimizingazimuthPos == true) { 
        //    SwerveModuleState optimizedState = SwerveModuleState.optimize(calculatedState, tempTurnOutput);
        //    set(driveOutput,optimizedState.angle.getRadians());//Units.metersToFeet(driveOutput),turnOutput);     
        //} else {
            set(-driveOutput, calculatedState.angle.getRadians());//Units.metersToFeet(driveOutput),turnOutput);
            // set(driveOutput, state.angle.getRadians());     
        //}   
        this.isAutonSwerveControllerOptimizingazimuthPos = false;
    }

    public double getVelocity_metersPerSec() {
        double sensoredVelInTicsPer100ms = driveController.getSelectedSensorVelocity(1);
        //SmartDashboard.putNumber("GetSensorVelocity", speed);
        double wheelCircumference = Units.inchesToMeters(DrivetrainConstants.WHEEL_DIAMETER_INCHES * Math.PI);
        double metersPer100ms = sensoredVelInTicsPer100ms * 1 * wheelCircumference / (SwervePodConstants2022.THRUST_ENCODER_UNITS_PER_REVOLUTION * 6.17);  //6.17 = gear ratio  <--should this be 6.17 here, or (1 / 6.17)?
        double metersPerSecond = metersPer100ms * 10 /*ms*/ / 1 /*sec*/;
        // SmartDashboard.putNumber("Velocity", metersPerSecond);
        return metersPerSecond;     
    }

    public SwerveModuleState getState() {
        state = new SwerveModuleState(getVelocity_metersPerSec(), new Rotation2d(tics2Rads(m_encoder.getPosition())));
                /*drivetrain.gyro.getRate() * DrivetrainConstants.DEGREES_PER_SECOND_TO_METERS_PER_SECOND_OF_WHEEL,
        drivetrain.getRotation2d());*/       
        return state;                                                                         //Not sure if this works
  }                   

    public void updateazimuthEncoder() {
        this.azimuthEncoderPosition = azimuthEncoder.getPosition();
        SmartDashboard.putNumber("P"+this.id+".azimuthEncoderPosition",this.azimuthEncoderPosition);

    }

 // public double getRate(){
 // }
}
