/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands.hscorer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import frc.robot.Robot;
import frc.robot.commands.Pause;

/**
 * Load a hatch from the loading station.
 * Extend the scorer, open the mechanism to grab the hatch,
 * and then retract the scorer so it's out of the way
 */
public class LoadHatch extends Command {
	private double pauseDur = 1;
	
	public LoadHatch() {
		requires(Robot.hScorer);
	}

	// Called every time the command starts
	@Override
	public void initialize() {
		System.out.println("Starting LoadHatch");
		
		// Push scorer
		Robot.hScorer.pushHS(true);
		// Pause for 1 second
		Scheduler.getInstance().add(new Pause(this.pauseDur));
		// Open scorer
		Robot.hScorer.openHS(true);
		// Pause for 1 second
		Scheduler.getInstance().add(new Pause(this.pauseDur));
		// Retract scorer
		Robot.hScorer.pushHS(false);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Nothing here, all done in initialize
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		// Not sure what we want here...
		return true;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println("Ending LoadHatch");
	}
}
