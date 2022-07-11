package team3176.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import team3176.robot.constants.IndexerConstants;

public class IndexerIOTalonSRX implements IndexerIO 
{
    private TalonSRX indexerMotor;

    public IndexerIOTalonSRX()
    {
        indexerMotor = new TalonSRX(IndexerConstants.INDEXER_CAN_ID);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {}

    @Override
    public void setVoltage(double volts) 
    {
        indexerMotor.set(TalonSRXControlMode.PercentOutput, volts);
    }

    @Override
    public void setIndexerPCT(double pct)
    {
        indexerMotor.set(ControlMode.PercentOutput, pct);
    }

}
