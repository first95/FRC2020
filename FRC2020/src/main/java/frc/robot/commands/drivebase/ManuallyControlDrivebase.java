package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManuallyControlDrivebase extends Command {

    public ManuallyControlDrivebase() {
        requires(Robot.drivebase);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.drivebase.driveWithJoysticks();
        Robot.drivebase.SetSuckerPower(Robot.oi.getSuckerAxis());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {

    }
}