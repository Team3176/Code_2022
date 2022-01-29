package team3176.robot.commands.auton;

import team3176.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CrossInitLine extends CommandBase {
    private Drivetrain drivetrain = Drivetrain.getInstance();

    public CrossInitLine() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        /*
        if(time stuff) {
            drivetrain.drive(some parameters)
        } else if(time stuff) {
            drivetrain.drive(other parameters)
        } else if(more time stuff) {
            driavetrain.state(vision);
            drivetrain.drive(parameters) //but it will stay locked on or something
            shooter.shoot()
        }
        */
    }

    @Override
    public boolean isFinished() {
        return false; //Would normally be a timer checking things
    }

    @Override
    public void end(boolean interrupted) { 

    }
}