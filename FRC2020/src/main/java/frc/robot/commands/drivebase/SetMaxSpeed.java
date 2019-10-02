package frc.robot.commands.drivebase;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class SetMaxSpeed extends Command {

	private double m_maxSpeed;
	
	public SetMaxSpeed(double maxSpeed) {
		m_maxSpeed = maxSpeed;
		
	}
	
	@Override
	protected void initialize() {
		super.initialize();
	
		Robot.drivebase.setMaxSpeed(m_maxSpeed);
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

}
