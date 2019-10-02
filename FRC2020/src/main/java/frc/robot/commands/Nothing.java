package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class Nothing extends Command {

	@Override
	protected boolean isFinished() {
		// This move sits and waits forever.
        // This is actually really useful at the end of certain moves.
        // If you want a commandgroup to run as a JoystickButton.whileHeld(), 
        // it will repeat while the button is held, unless you end that commandgroup
        // with one of these.
		return false;
	}

}
