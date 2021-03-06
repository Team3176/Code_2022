package team3176.robot.util.XboxController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Sets up a extention of a Trigger object where it can have one of the two modifiers active to run the shift key
 */

public class XboxShift extends Trigger {
    private final XboxController m_Controller;
    private final int calledButton; 
    private final int primaryShiftKey;
    private final int secondaryShiftKey;

    /**
     * @param control The XboxController being used
     * @param button The key
     * @param shiftKey The shift key (in the case of this code it is Left Bumper)
     * @param DBLShiftKey The additional shift key (in the case of this code it is Right Bumper)
     */

    public XboxShift(XboxController control, int button, int shiftKey, int DBLShiftKey) {
        this.m_Controller = control;
        this.calledButton = button;
        this.primaryShiftKey = shiftKey;
        this.secondaryShiftKey = DBLShiftKey;
    }

    @Override
    public boolean get() {
        if(m_Controller.getRawButton(calledButton) && m_Controller.getRawButton(primaryShiftKey) && !m_Controller.getRawButton(secondaryShiftKey)) 
        {return true;}
        return false;
    }
}