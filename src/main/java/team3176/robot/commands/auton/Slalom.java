package team3176.robot.commands.auton;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import team3176.robot.subsystems.drivetrain.CoordSys;
import team3176.robot.subsystems.drivetrain.CoordSys.coordType;
import team3176.robot.subsystems.drivetrain.Gyro3176;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Slalom extends CommandBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();
    private CoordSys coordSys = CoordSys.getInstance();
    private Gyro3176 gyro = Gyro3176.getInstance();
    private double startTime;
    double runTimeInput;
    double time1 = 1.15;
    double time2 = 1.43;
    double time3 = 3.5; // It's about that but still need to est

    public Slalom() {
        addRequirements(drivetrain);
        // runTimeInput = SmartDashboard.getNumber("runTime", 0.5);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        coordSys.setCoordType(coordType.FIELD_CENTRIC);
        gyro.resetGyro();
    }

    @Override
    public void execute() {
        if((startTime + time1) > Timer.getFPGATimestamp()) {
            drivetrain.drive(0.5, 0.0, 0.0);
        } else if((startTime + time1 + time2) > Timer.getFPGATimestamp()) {
            drivetrain.drive(0.0, -0.5, 0.0);
        // } else if((startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) > Timer.getFPGATimestamp()) {
            // drivetrain.drive(0.5, 0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // return (startTime + time1 + time2 + SmartDashboard.getNumber("runTime", runTimeInput)) < Timer.getFPGATimestamp();
        return true;
    }

    @Override
    public void end(boolean interrupted) { 
        drivetrain.drive(0.0, 0.0, 0.0);
    }
}