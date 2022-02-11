package team3176.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3176.robot.constants.ClimbPassiveConstants;

public class ClimbPassive extends SubsystemBase {
    private static ClimbPassive instance = new ClimbPassive();
    public static ClimbPassive getInstance() {return instance;}

    DoubleSolenoid passiveClimbPiston1;
    DoubleSolenoid passiveClimbPiston2;

    public ClimbPassive() {
        passiveClimbPiston1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbPassiveConstants.PASSIVE_PISTON_ONE_OPEN_ID, ClimbPassiveConstants.PASSIVE_PISTON_ONE_CLOSE_ID);
        passiveClimbPiston2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbPassiveConstants.PASSIVE_PISTON_TWO_OPEN_ID, ClimbPassiveConstants.PASSIVE_PISTON_TWO_CLOSE_ID);
    }

    /**
     * Extends both of the Pistons on PassiveClimb
     */

    public void extendPistons() {
        passiveClimbPiston1.set(Value.kForward);
        passiveClimbPiston2.set(Value.kForward);
    }

    /**
     * Retracts both of the Pistons on PassiveClimb
     */

    public void retractPistons() {
        passiveClimbPiston1.set(Value.kReverse);
        passiveClimbPiston2.set(Value.kReverse);
    }
}
