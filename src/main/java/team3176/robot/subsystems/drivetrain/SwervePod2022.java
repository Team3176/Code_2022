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


public class SwervePod2022 {

    /** Class Object holding the Motor controller for Drive Motor on the SwervePod */
    private TalonFX driveController;
    /** Class Object holding the Motor controller for Spin (aka Azimuth) Motor on the SwervePod */
    private CANSparkMax spinController;
    /** Class Object holding the CAN-based encoder for Spin (aka Azimuth) position of the SwervePod */
    CANCoder spinEncoder;
    /** Current value in radians of the spinEncoder's position */
    double spinEncoderPosition;

    /** Numerical identifier to differentiate between pods.
     *     For 4 Pods:  0 = FrontRight (FR),
     *                  1 = FrontLeft (FL),
     *                  2 = BackLeft (BL),
     *                  3 = BackRight (BR)
     */
    private int id;
   
    /** Represents the value of the spinEncoder reading in radians when positioned with the positive Thurst vector of the Pod's Drive wheel pointing towards front of robot */
    private double kEncoderOffset; 
    //private double kSpinEncoderUnitsPerRevolution;
    private double kDriveEncoderUnitsPerRevolution;
    private int off = 0;

    private double lastEncoderPos;
    private double radianError;
    private double radianPos;
    private double encoderError;
    private double encoderPos;

    private double azimuthCommand;
    private double velTicsPer100ms;

    public int kSlotIdx_spin, kPIDLoopIdx_spin, kTimeoutMs_spin,kSlotIdx_drive, kPIDLoopIdx_drive, kTimeoutMs_drive;

    private double podDrive, podSpin;

    private double kP_Spin;
    private double kI_Spin;
    private double kD_Spin;
    private double kFF_Spin;
    private double kIz_Spin;
    private double kMaxOutput;
    private double kMinOutput;

    private double kSpinRampRate;

    private double kP_Drive;
    private double kI_Drive;
    private double kD_Drive;
    private double kF_Drive;

    private double maxVelTicsPer100ms;
    private boolean isAutonSwerveControllerOptimizingSpinPos = false;

    private double PI = Math.PI;
    private double maxFps = SwervePodConstants2022.DRIVE_SPEED_MAX_EMPIRICAL_FEET_PER_SECOND;

    private double startTics;

    private final PIDController m_drivePIDController;
    private final ProfiledPIDController m_turningPIDController;
    //private ProfiledPIDController m_turningPIDController;
    private SwerveModuleState state;
    private RelativeEncoder m_encoder;

    public SwervePod2022(int id, TalonFX driveController, CANSparkMax spinController) {
        this.id = id;
        spinEncoder = new CANCoder(SwervePodConstants2022.STEER_CANCODER_CID[id]);
        updateSpinEncoder();
        initializeSmartDashboard();
        
        this.kEncoderOffset = Math.toRadians(SwervePodConstants2022.SPIN_OFFSET[this.id]);
        ///System.out.println("P"+(this.id+1)+" kEncoderOffset: "+this.kEncoderOffset);

        //kSpinEncoderUnitsPerRevolution = SwervePodConstants2022.SPIN_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_spin = SwervePodConstants2022.TALON_SPIN_PID_SLOT_ID;
        kPIDLoopIdx_spin = SwervePodConstants2022.TALON_SPIN_PID_LOOP_ID;
        kTimeoutMs_spin = SwervePodConstants2022.TALON_SPIN_PID_TIMEOUT_MS;
        
        kP_Drive = 0.03; // SwervePodConstants.DRIVE_PID[0][id];
        kI_Drive = 0.0; // SwervePodConstants.DRIVE_PID[1][id];
        kD_Drive = 0.0; // SwervePodConstants.DRIVE_PID[2][id];
        kF_Drive = .045; // SwervePodConstants.DRIVE_PID[3][id];


        m_drivePIDController = new PIDController(SwervePodConstants2022.P_MODULE_DRIVE_CONTROLLER, 0, 0);

        kP_Spin = SwervePodConstants2022.SPIN_PID[0][id];
        kI_Spin = SwervePodConstants2022.SPIN_PID[1][id];
        kD_Spin = SwervePodConstants2022.SPIN_PID[2][id];
        kFF_Spin = SwervePodConstants2022.SPIN_PID[3][id];
        kIz_Spin = SwervePodConstants2022.SPIN_PID[4][id];
        kMaxOutput = SwervePodConstants2022.SPIN_PID[5][id];
        kMinOutput = SwervePodConstants2022.SPIN_PID[6][id];

        kP_Spin = 0.0;
        kI_Spin = 0.0;
        kD_Spin = 0.0;
        m_turningPIDController = new ProfiledPIDController(
            kP_Spin, kI_Spin, kD_Spin, 
            new TrapezoidProfile.Constraints(
                (SwervePodConstants2022.MAX_MODULE_ANGULAR_SPEED_RADIANS_PER_SECOND),
                (SwervePodConstants2022.MAX_MODULE_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED)));
        
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_turningPIDController.reset(0.0, 0.0);
        

        /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
        //spinController.configAllowableClosedloopError(0, SwervePodConstants.kPIDLoopIdx, SwervePodConstants.kTimeoutMs);

        kDriveEncoderUnitsPerRevolution = SwervePodConstants2022.DRIVE_ENCODER_UNITS_PER_REVOLUTION;
        kSlotIdx_drive = SwervePodConstants2022.TALON_DRIVE_PID_SLOT_ID;
        kPIDLoopIdx_drive = SwervePodConstants2022.TALON_DRIVE_PID_LOOP_ID;
        kTimeoutMs_drive = SwervePodConstants2022.TALON_DRIVE_PID_TIMEOUT_MS;

        m_encoder = spinController.getEncoder();

        this.driveController = driveController;
        this.spinController = spinController;
        //this.spinPIDController = spinController.getPIDController();

        this.driveController.configFactoryDefault();
        //this.spinController.restoreFactoryDefaults();
        this.spinController.setOpenLoopRampRate(10);
        //this.spinController.burnFlash();

        this.driveController.configClosedloopRamp(0.5);    

       // this.driveController.setNeutralMode(NeutralMode.Brake);
       // this.driveController.setNeutralMode(NeutralMode.Brake);

        this.driveController.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        //this.spinController.configSelectedFeedbackSensor(FeedbackDevice.m_encoder, 0, 0);   //TODO: investigate QuadEncoder vs CTRE_MagEncoder_Absolute.  Are the two equivalent?  Why QuadEncoder instead of CTRE_MagEncoder_Absolute

        /*
        if (this.id == 0 || this.id == 1) {
            this.spinController.setSensorPhase(SwervePodConstants2022.kSensorPhase);
            this.spinController.setInverted(SwervePodConstants2022.kMotorInverted);
        }
        if (this.id == 2 || this.id == 3) {
            this.spinController.setSensorPhase(true);
            this.spinController.setInverted(true);
        }
        */

            //TODO: check out "Feedback Device Not Continuous"  under config tab in CTRE-tuner.  Is the available via API and set-able?  Caps encoder to range[-4096,4096], correct?
                //this.spinController.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition), 0, 0);
                //this.spinController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute), 0, 0);

        this.driveController.config_kP(kPIDLoopIdx_drive, kP_Drive, kTimeoutMs_drive);
        this.driveController.config_kI(kPIDLoopIdx_drive, kI_Drive, kTimeoutMs_drive);
        this.driveController.config_kD(kPIDLoopIdx_drive, kD_Drive, kTimeoutMs_drive);
        this.driveController.config_kF(kPIDLoopIdx_drive, kF_Drive, kTimeoutMs_drive);

        /*
        this.spinPIDController.setP(kP_Spin);
        this.spinPIDController.setI(kI_Spin);
        this.spinPIDController.setD(kD_Spin);
        this.spinPIDController.setFF(kFF_Spin);
        this.spinPIDController.setIZone(kIz_Spin);
        this.spinPIDController.setOutputRange(kMinOutput,kMaxOutput);
        */

        /*
        this.spinPIDController.setP(kP_Spin, kPIDLoopIdx_spin);
        this.spinPIDController.setI(kI_Spin, kPIDLoopIdx_spin);
        this.spinPIDController.setD(kD_Spin, kPIDLoopIdx_spin);
        this.spinPIDController.setFF(kFF_Spin, kPIDLoopIdx_spin);
        this.spinPIDController.setIZone(kIz_Spin, kPIDLoopIdx_spin);
        this.spinPIDController.setOutputRange(kMinOutput, kMaxOutput);
        */

    }

    public void tune() {
        this.kP_Spin = SmartDashboard.getNumber("P"+(this.id)+".kP_Spin",0);
        this.kI_Spin = SmartDashboard.getNumber("P"+(this.id)+".kI_Spin",0);
        this.kD_Spin = SmartDashboard.getNumber("P"+(this.id)+".kD_Spin",0);
        m_turningPIDController.setP(this.kP_Spin);
        m_turningPIDController.setI(this.kP_Spin);
        m_turningPIDController.setD(this.kP_Spin);
        SmartDashboard.putNumber("P"+(this.id)+".kP_Spin",this.kP_Spin);
        SmartDashboard.putNumber("P"+(this.id)+".kI_Spin",this.kI_Spin);
        SmartDashboard.putNumber("P"+(this.id)+".kD_Spin",this.kD_Spin);
        
        
        this.kSpinRampRate = SmartDashboard.getNumber("P"+(this.id)+".kRampRate_Spin", 0);

        
        

        this.spinController.setOpenLoopRampRate(this.kSpinRampRate);
        //this.spinController.setSmartCurrentLimit(20);
        //this.spinController.burnFlash();

        this.podSpin = SmartDashboard.getNumber("P"+(this.id)+".podSpin_setpoint", 0);

        //this.podSpin = 0.2;
        //System.out.println("podSpin = "+this.podSpin);

        double optmizdSpinPos = optimizeSpinPos(this.podSpin);

        SmartDashboard.putNumber("P"+(this.id)+".optmizdSpinPos", optmizdSpinPos);

        double turnOutput = m_turningPIDController.calculate(this.spinEncoderPosition, optmizdSpinPos);

        SmartDashboard.putNumber("P"+(this.id)+".turnOutput", turnOutput);


    }
    /**
     * @param podDrive represents desired thrust of swervepod Range = -1 to 1 or
     *                 ft-per-sec?
     * @param podSpin  represents desired angle of swervepod. Range = -pi to pi.
     */
    public void set(double podDrive, double podSpin) {
        this.podDrive = podDrive;
        this.podSpin = podSpin; 
         //this.spinController.config_kP(kSlotIdx_spin, SmartDashboard.getNumber("P", kP_Spin), kTimeoutMs_spin);
         //this.spinController.config_kI(kSlotIdx_spin, SmartDashboard.getNumber("I", kI_Spin), kTimeoutMs_spin);
         //this.spinController.config_kD(kSlotIdx_spin, SmartDashboard.getNumber("D", kD_Spin), kTimeoutMs_spin);
         //this.spinController.config_kF(kSlotIdx_spin, SmartDashboard.getNumber("F", kF_Spin), kTimeoutMs_spin);
        // this.driveController.config_kP(kSlotIdx_drive, SmartDashboard.getNumber("P", kP_Drive), kTimeoutMs_spin);
        // this.driveController.config_kI(kSlotIdx_drive, SmartDashboard.getNumber("I", kI_Drive), kTimeoutMs_spin);
        // this.driveController.config_kD(kSlotIdx_drive, SmartDashboard.getNumber("D", kD_Drive), kTimeoutMs_spin);
        // this.driveController.config_kF(kSlotIdx_drive, SmartDashboard.getNumber("F", kF_Drive), kTimeoutMs_spin);
        

            // TODO: need check ether output values. speed vs %-values
        // this.maxVelTicsPer100ms = 1 * 987.2503 * kDriveEncoderUnitsPerRevolution / 600.0;
        // this.velTicsPer100ms = this.podDrive * 2000.0 * kDriveEncoderUnitsPerRevolution / 600.0;  //TODO: rework "podDrive * 2000.0"
        this.maxVelTicsPer100ms = fps2ums(DrivetrainConstants.MAX_WHEEL_SPEED_FEET_PER_SECOND);
        this.velTicsPer100ms = fps2ums(this.podDrive);
       // velTicsPer100ms = SmartDashboard.getNumber("driveSet",velTicsPer100ms);
        double optmizdSpinPos = optimizeSpinPos(this.podSpin);
        //double tics = rads2Tics(this.podSpin);

        final double turnOutput = m_turningPIDController.calculate(this.spinEncoderPosition, optmizdSpinPos);

        //if (this.id == 3) {spinController.set(ControlMode.Position, 0.0); } else {   // TODO: Try this to force pod4 to jump lastEncoderPos
        if (this.podDrive > (-Math.pow(10,-10)) && this.podDrive < (Math.pow(10,-10))) {      //TODO: convert this to a deadband range.  abs(podDrive) != 0 is notationally sloppy math
            //spinController.set(turnOutput * SwervePodConstants2022.SPIN_SPARKMAX_MAX_OUTPUTPERCENT);
            SmartDashboard.putNumber("P"+(id) + " turnOutput",turnOutput);
            //spinPIDController.setReference(this.encoderPos, CANSparkMax.ControlType.kPosition);  
        } else {
            //spinController.set(turnOutput * SwervePodConstants2022.SPIN_SPARKMAX_MAX_OUTPUTPERCENT);
            SmartDashboard.putNumber("P"+(id) + " turnOutput",turnOutput);
            //spinPIDController.setReference(this.encoderPos, CANSparkMax.ControlType.kPosition);  
            this.lastEncoderPos = optmizdSpinPos;
        }    

        driveController.set(TalonFXControlMode.Velocity, velTicsPer100ms);
        }
    

    /**
     * @param angle desired angle of swerve pod in units of radians, range from -PI to +PI
     * @return
     */
    private double optimizeSpinPos(double angle) {
        //System.out.println("calcSpinPos - P"+(this.id+1)+" kEncoderOffset: "+this.kEncoderOffset);

        this.encoderPos = getEncoderPos();;
        //this.encoderPos = spinController.getSelectedSensorPosition() - this.kEncoderOffset;
        //radianPos = tics2Rads(this.encoderPos);
        radianPos = (this.encoderPos);
        radianError = angle - radianPos;
        // FYI: Math.copySign(magnitudeVar, signVar) = magnitude value with same sign as signvar

        //if (Math.abs(radianError) > (5 * (PI / 2))) {
        //    System.out.println("Error: Overload");
        //} else if (Math.abs(radianError) > (3 * (PI / 2))) {

        if (isAutonSwerveControllerOptimizingSpinPos == false) { 
            if (Math.abs(radianError) > (3 * (PI / 2))) {      // TODO: See if commenting out "Thrust-vector sign-flip" fixes
                radianError -= Math.copySign(2 * PI, radianError);
            } else if (Math.abs(radianError) > (PI / 2)) {
                radianError -= Math.copySign(PI, radianError);
                this.velTicsPer100ms = -this.velTicsPer100ms;
            }
        }
        //encoderError = rads2Tics(radianError);
        encoderError = (radianError);
        azimuthCommand = encoderError + this.encoderPos + this.kEncoderOffset;
        return (azimuthCommand);
    }

    public void goHome() {
        double homePos = 0 + this.kEncoderOffset;
        //this.spinPIDController.setReference(homePos, CANSparkMax.ControlType.kPosition);

    }

    /*
    private int rads2Tics(double rads) {        //TODO: put a modulo cap limit like in tics2Rads (range[-pi,pi])  (Is it returning 0-2pi somehow?)
        //rads = rads * (2 * Math.PI);
        double rads_clamped = (Math.min((Math.max(rads,(-Math.PI))), (Math.PI)));        
        double tics = ((rads_clamped / (2.0*Math.PI)) * kSpinEncoderUnitsPerRevolution);
        return (int) tics;
    }
    */

    /*
    private double tics2Rads(double tics) {
        tics = tics % kSpinEncoderUnitsPerRevolution;
        if(tics < 0) {
            tics += kSpinEncoderUnitsPerRevolution;
        }
        tics -= (kSpinEncoderUnitsPerRevolution / 2);
        return ((tics / kSpinEncoderUnitsPerRevolution) * (2 * PI));
    }
    */

    /**
     * @param i feet per second
     * @return tics per 100ms
     */
    private double fps2ums(double i) {
        
        // input * inchesPerFoot * circumfrenceOfWheel * ticsPerRev * gearRatio * secTo100ms
        return i * 12.0 * (1.0/10.21) * 2048.0 *6.17 * .1;
        
        // return i * 100;
    }

    public boolean isInverted() { return spinController.getInverted(); }
    public void setInverted() { spinController.setInverted(!isInverted()); }

    public void updateSpinEncoder() {
        this.spinEncoderPosition = spinEncoder.getPosition();
        SmartDashboard.putNumber("P"+this.id+".spinEncoderPosition",this.spinEncoderPosition);

    }

    public double getEncoderPos() {
        updateSpinEncoder();
        return this.spinEncoderPosition;
    }

    public void initializeSmartDashboard() {

        SmartDashboard.putNumber("P"+(this.id)+".podSpin_setpoint_angle", 0);
        SmartDashboard.putNumber("P"+(this.id)+".kP_Spin", 0);
        SmartDashboard.putNumber("P"+(this.id)+".kI_Spin", 0);
        SmartDashboard.putNumber("P"+(this.id)+".kD_Spin", 0);
        SmartDashboard.putNumber("P"+(this.id)+".kRampRate_Spin", 0);
        SmartDashboard.putNumber("P"+(this.id)+".podSpin_encoder", this.spinEncoderPosition);
        SmartDashboard.putBoolean("P"+(this.id)+".On", false);
    }


    public void SwervePod2022SmartDashboardComments () {
        //SwervePod2022 comments start
        // SmartDashboard.putNumber("P", kP_Drive);
        // SmartDashboard.putNumber("I", kI_Drive);
        // SmartDashboard.putNumber("D", kD_Drive);
        // SmartDashboard.putNumber("F", kF_Drive);
       // SmartDashboard.putNumber("driveSet",0);       
       
        // SmartDashboard.putNumber("P" + (id + 1) + " podDrive", this.podDrive);

        // SmartDashboard.putNumber("P" + (id + 1) + " podSpin", this.podSpin);
        //SwervePod2022 comments end


        //set(double podDrive, double podSpin) comments start
        // SmartDashboard.putNumber("P" + (id + 1) + " podDrive", this.podDrive);
        // SmartDashboard.putNumber("P" + (id + 1) + " podSpin", this.podSpin);

        // SmartDashboard.putNumber("fps2ums:velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("podDrive", this.podDrive);

        // SmartDashboard.putNumber("P" + (id + 1) + " tics", tics);
        // SmartDashboard.putNumber("P" + (id + 1) + " absTics", spinController.getSelectedSensorPosition());

        // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " lastEncoderPos", this.lastEncoderPos);

        //SmartDashboard.putNumber("P" + (id) + "getSelSenPos", spinController.getSelectedSensorPosition());

        // SmartDashboard.putNumber("P"+this.id+".podDrive", podDrive);
        //SmartDashboard.putNumber("actualVel", driveController.getVoltage());
        
        // if (this.id == 0){
        //     SmartDashboard.putNumber("Pod3Distance", driveController.getSelectedSensorPosition());
        //     SmartDashboard.putNumber("velTicsPer100ms", velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error", driveController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("Tics Error2", driveController.getSelectedSensorVelocity()-velTicsPer100ms);
        //     SmartDashboard.putNumber("DriveController Velocity", driveController.getSelectedSensorVelocity());
        // } 
        
        // SmartDashboard.putNumber("P" + (id + 1) + " velTicsPer100ms", velTicsPer100ms);
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderSetPos_end", encoderSetPos);
        //}
        //set(double podDrive, double podSpin) comments start


        //private double optimizeSpinPos(double angle) comments start
        // SmartDashboard.putNumber("P" + (id + 1) + " calcSpinPos_angle", angle);
        
        // SmartDashboard.putNumber("P" + (id + 1) + " kEncoderOffset", this.kEncoderOffset);
        // SmartDashboard.putNumber("P" + (id + 1) + " getSelectedSensorPosition", spinController.getSelectedSensorPosition());
        // SmartDashboard.putNumber("P" + (id + 1) + " encoderPos_in_calcSpinPos",this.encoderPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " radianPos", radianPos);

        // SmartDashboard.putNumber("P" + (id + 1) + " radianError", radianError);

        // SmartDashboard.putNumber("P" + (id + 1) + " encoderError", encoderError);

        // SmartDashboard.putNumber("P" + (id + 1) + "tics2radianDrivecommand", driveCommand);
        //private double optimizeSpinPos(double angle) comments end


        //public void setDesiredState(SwerveModuleState desiredState) comments start
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_desiredDegrees", desiredState.angle.getDegrees());
        // SmartDashboard.putNumber("P"+this.id+".setdesiredState_sensoredDegrees", rotation.getDegrees());

        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_TurnOutput",turnOutput);
        // SmartDashboard.putNumber("P"+this.id+".setDesiredState_DriveOutput",driveOutput);
        //public void setDesiredState(SwerveModuleState desiredState) comments ends


        //public double getVelocity_metersPerSec() comments start
        //SmartDashboard.putNumber("GetSensorVelocity", speed);
        // SmartDashboard.putNumber("Velocity", metersPerSecond);
        //public double getVelocity_metersPerSec() comments end
        
    }

 // public double getRate(){
 // }
}
