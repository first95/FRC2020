package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ManuallyControlDrivebase extends CommandBase {

    public ManuallyControlDrivebase() {
        addRequirements(Robot.drivebase);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        Robot.drivebase.driveWithJoysticks();
        Robot.drivebase.SetSuckerPower(Robot.oi.getSuckerAxis());
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}