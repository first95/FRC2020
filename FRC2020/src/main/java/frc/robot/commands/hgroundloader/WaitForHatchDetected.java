package frc.robot.commands.hgroundloader;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that waits until the intake sees a current spike indicating the presence of a hatch
 */
public class WaitForHatchDetected extends Command {
	public WaitForHatchDetected() {
        requires(Robot.hGroundLoader);
	}
	
	@Override
	protected boolean isFinished() {
		return Robot.hGroundLoader.getIntakeCurrentSpike();
	}

}