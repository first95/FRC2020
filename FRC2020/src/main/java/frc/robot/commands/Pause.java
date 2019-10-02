package frc.robot.commands;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class Pause extends TimedCommand {
	// This move waits and does nothing for a specified duration
	public Pause(double pauseDurationS) {
		super(pauseDurationS);
    }
    
    @Override
    public synchronized void start() {
        System.out.println("Pause.start()");
        super.start();
    }

    @Override
    protected void end() {
        System.out.println("Pause.end()");
        super.end();
    }
}
