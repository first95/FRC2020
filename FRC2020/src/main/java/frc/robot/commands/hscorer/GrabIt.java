package frc.robot.commands.hscorer;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * Set the hatch scorer to open or closed, then wait a short duration for it to actuate
 */
public class GrabIt extends TimedCommand {
    private static final double DEFAULT_DURATION_S = 0.3;
    private boolean grab;
	
	public GrabIt(boolean grab, double durationS) {
        super(durationS);
        requires(Robot.hScorer);
        this.grab = grab;
	}

	public GrabIt(boolean grab) {
        this(grab, DEFAULT_DURATION_S);
	}

	@Override
	public void execute() {
		super.execute();
		Robot.hScorer.openHS(!grab);
	}
}
