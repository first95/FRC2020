package frc.robot.commands.hscorer;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Robot;

/**
 * Set the hatch scorer to extended or retracted
 */
public class PushIt extends TimedCommand {
    private static final double DEFAULT_DURATION_S = 0.3;
    private boolean push;

    public PushIt(boolean push, double durationS) {
        super(durationS);
        requires(Robot.hScorer);
        this.push = push;
    }

    public PushIt(boolean push) {
        this(push, DEFAULT_DURATION_S);
    }

    @Override
    public void execute() {
        super.execute();
        Robot.hScorer.pushHS(push);
    }
}
