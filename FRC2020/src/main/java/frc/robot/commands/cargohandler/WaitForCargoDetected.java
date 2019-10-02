package frc.robot.commands.cargohandler;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that waits until the intake sees a current spike indicating the presence of a hatch
 */
public class WaitForCargoDetected extends Command {
	public WaitForCargoDetected() {
        requires(Robot.cargoHandler);
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.cargoHandler.getIntakeCurrentSpike();
	}

}