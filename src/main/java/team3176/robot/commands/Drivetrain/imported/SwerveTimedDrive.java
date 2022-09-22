package team3176.robot.commands.Drivetrain.imported;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3176.robot.subsystems.SwerveSubsystem.*;
import team3176.robot.subsystems.SwerveSubsystem.SwerveSubsystem.driveMode;


public class SwerveTimedDrive  extends CommandBase{
    double forwardCommand, strafeCommand, spinCommand;
    SwerveSubsystem SwerveSubsystem = SwerveSubsystem.getInstance();
    CoordSys m_CoordSys = CoordSys.getInstance();
    Gyro3176 m_gyro = Gyro3176.getInstance();
    /**
     * For use in Autonmous. Call with the decoration ".withTimeout(seconds)" 
     *  example: SwerveTimedDrive(0.1,0.5,1.2).withTimeout(2);
     * @param forwardCommand ft/s
     * @param strafeCommand ft/s
     * @param spinCommand rad/s
     */
    public SwerveTimedDrive(Double forwardCommand, Double strafeCommand, Double spinCommand) {
        this.forwardCommand = forwardCommand;
        this.strafeCommand = strafeCommand;
        this.spinCommand = spinCommand;
        addRequirements(SwerveSubsystem);
        addRequirements(m_CoordSys);
        }
    
        @Override
        public void initialize() {
            SwerveSubsystem.setDriveMode(driveMode.DRIVE);
            m_gyro.setSpinLockToOff();
        }
    
        @Override
        public void execute() {
        SwerveSubsystem.drive(forwardCommand, strafeCommand, spinCommand, m_CoordSys.getCurrentCoordType());
        }
    
        @Override
        public boolean isFinished() { return false; }
    
        @Override
        public void end(boolean interrupted) {  
            SwerveSubsystem.drive(0.0, 0.0, 0.0, m_CoordSys.getCurrentCoordType());
        }
}
