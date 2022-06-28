package team3176.robot.subsystems.flywheel;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import team3176.robot.constants.FlywheelConstants;
import team3176.robot.util.God.Units3176;

public class FlywheelIOTalonFX implements FlywheelIO
{
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;
    private boolean isFlywheelSpinning;

    public FlywheelIOTalonFX()
    {
        flywheelMotor1 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON1_CAN_ID);
        flywheelMotor2 = new TalonFX(FlywheelConstants.FLYWHEEL_FALCON2_CAN_ID);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) 
    {
        isFlywheelSpinning = false;
    }

    public void setFlywheelVelocity(double pctOne, double pctTwo)
    {
        flywheelMotor1.set(TalonFXControlMode.Velocity, pctOne * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
        flywheelMotor2.set(TalonFXControlMode.Velocity, pctTwo * Units3176.revolutionsPerMinute2ticsPer100MS(6380, 2048));
        isFlywheelSpinning = true;
        if((pctOne == 0) && (pctTwo == 0)) {isFlywheelSpinning = false;}
    }
}
