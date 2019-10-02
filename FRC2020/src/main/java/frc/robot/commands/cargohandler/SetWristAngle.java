package frc.robot.commands.cargohandler;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class SetWristAngle extends Command {
	private Double targetDegrees = 0.0;

	public SetWristAngle(double targetDegrees) {
		// This method is run once during robot startup
		requires(Robot.cargoHandler);
		this.targetDegrees = targetDegrees;
	}

	@Override
	public synchronized void initialize() {
		Robot.cargoHandler.setWristAngleDeg(targetDegrees);
	}
	
	@Override
	protected void execute() {
		// This method is called every iteration
		
		// Nothing needed; we did everything we needed in initialize()
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.cargoHandler.isOnTarget();
	}
	
}
