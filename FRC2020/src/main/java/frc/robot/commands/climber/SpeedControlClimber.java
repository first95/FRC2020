package frc.robot.commands.climber;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class SpeedControlClimber extends Command {
	
	public SpeedControlClimber() {
		requires(Robot.climber);
	}

	@Override
	protected void execute() {
		Robot.climber.setSpeed(Robot.oi.getClimberSpeed());
		if (Robot.oi.isDeploySkidsToggled()) {
			Robot.climber.toggleSkids();
		}
	}

	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

}