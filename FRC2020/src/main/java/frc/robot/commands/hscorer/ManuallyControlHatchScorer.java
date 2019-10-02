package frc.robot.commands.hscorer;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ManuallyControlHatchScorer extends Command {
	
	public ManuallyControlHatchScorer() {
		requires(Robot.hScorer);
	}

	@Override
	protected void execute() {
		if (Robot.oi.isGrabHatchButtonHeld()) {
			Robot.hScorer.openHS(true);
		}
		else
		{
			Robot.hScorer.openHS(false);
		}
		if (Robot.oi.isPushHatchButtonHeld()) {
			Robot.hScorer.pushHS(true);
		}
		else
		{
			Robot.hScorer.pushHS(false);
		}
	}

	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

}