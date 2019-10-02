package frc.robot.commands.brakes;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class BrakesCommand extends Command {
	
	public BrakesCommand() {
		requires(Robot.brakes);
	}

	@Override
	protected void execute() {
		if (Robot.oi.isBrakesButtonHeld()) {
			Robot.brakes.setBrakes(true);
		}
		else
		{
			Robot.brakes.setBrakes(false);
		}
	}

	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

}