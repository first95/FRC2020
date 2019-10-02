package frc.robot.commands.cargohandler;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpeedControlCargoHandler extends Command {
	private double intakeSpeed;
	private double wristSpeed;
	
	public SpeedControlCargoHandler() {
		requires(Robot.cargoHandler);
	}

	@Override
	protected void execute() {
		intakeSpeed = Robot.oi.getCargoHandlerIntakeSpeed();
		Robot.cargoHandler.setIntakeSpeed(intakeSpeed);
		SmartDashboard.putNumber("Cargo Handler Intake Input", intakeSpeed);

		wristSpeed = Robot.oi.getCargoHandlerWristSpeed();
		Robot.cargoHandler.setWristPitchSpeed(wristSpeed);
		SmartDashboard.putNumber("Cargo Handler Wrist Input", wristSpeed);
	}
	
	@Override
	protected boolean isFinished() {
		return false; // until interrupted
	}

}