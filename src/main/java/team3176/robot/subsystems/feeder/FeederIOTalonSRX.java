package team3176.robot.subsystems.feeder;

import team3176.robot.constants.FeederConstants;
import team3176.robot.util.God.Units3176;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class FeederIOTalonSRX implements FeederIO 
{
    private TalonSRX feederMotor;
    private boolean isFeederRunning = false;

    public FeederIOTalonSRX()
    {
        feederMotor = new TalonSRX(FeederConstants.FEEDER_MOTOR_CAN_ID);
        feederMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0); 
        this.feederMotor.configNominalOutputForward(0, FeederConstants.kTIMEOUT_MS);
        this.feederMotor.configNominalOutputReverse(0, FeederConstants.kTIMEOUT_MS);
        this.feederMotor.configPeakOutputForward(1.0, FeederConstants.kTIMEOUT_MS);
        this.feederMotor.configPeakOutputReverse(-1.0, FeederConstants.kTIMEOUT_MS);
        this.feederMotor.configAllowableClosedloopError(FeederConstants.kPID_LOOP_IDX, FeederConstants.ALLOWABLE_CLOSED_LOOP_ERROR, FeederConstants.kTIMEOUT_MS);
        this.feederMotor.config_kF(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][3], FeederConstants.kTIMEOUT_MS);
        this.feederMotor.config_kP(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][0], FeederConstants.kTIMEOUT_MS);
        this.feederMotor.config_kI(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][1], FeederConstants.kTIMEOUT_MS);
        this.feederMotor.config_kD(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][2], FeederConstants.kTIMEOUT_MS);
        this.feederMotor.config_IntegralZone(FeederConstants.kPID_LOOP_IDX, FeederConstants.PIDFConstants[0][4], FeederConstants.kTIMEOUT_MS);
        this.feederMotor.setInverted(true);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {}

    @Override
    public void setFeederVelocity(double pctAsDecimal)
    {
        feederMotor.set(ControlMode.Velocity, Units3176.revolutionsPerMinute2ticsPer100MS(18730, 4096));
        isFeederRunning = true;
        if(pctAsDecimal == 0) {isFeederRunning = false;}
    }

    @Override
    public void setFeederPCT(double pct)
    {
        feederMotor.set(ControlMode.PercentOutput, pct);
    }

    @Override
    public void setVoltage(double volts)
    {

    }

}
