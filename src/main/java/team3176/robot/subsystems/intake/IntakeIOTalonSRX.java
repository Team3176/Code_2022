package team3176.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import team3176.robot.constants.IntakeConstants;

public class IntakeIOTalonSRX implements IntakeIO
{
    private TalonSRX intakeMotor;
    private DoubleSolenoid piston;

    public IntakeIOTalonSRX()
    {
        intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN_ID);
        piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DSOLENOID1_FWD_CHAN, IntakeConstants.DSOLENOID1_REV_CHAN);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.isExtend = piston.get() == Value.kReverse;
    }

    @Override
    public void setVoltage(double volts) 
    {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, volts);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) 
    {
        intakeMotor.set(TalonSRXControlMode.Velocity, velocityRadPerSec);
    }

    @Override
    public void setPiston(boolean isExtend) 
    {
        piston.set(isExtend ? Value.kForward : Value.kReverse);
    }
}   