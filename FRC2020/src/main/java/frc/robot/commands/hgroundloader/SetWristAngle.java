package frc.robot.commands.hgroundloader;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class SetWristAngle extends Command {
	private double targetDegrees = 0.0;
	private boolean waitForWristToReachTarget;

	public SetWristAngle(double target, boolean wait) {
		// This method is run once during robot startup
		requires(Robot.hGroundLoader);
		targetDegrees = target;
		waitForWristToReachTarget = wait;
	}

	@Override
	public synchronized void start() {
        System.out.println("SetWristAngle.start()");
        super.start();
	}
	
	@Override
	protected void execute() {
		// This method is called every iteration
		Robot.hGroundLoader.setWristAngleDeg(targetDegrees);
	}
	
	@Override
	protected boolean isFinished() {
		boolean finished = (!waitForWristToReachTarget) || Robot.hGroundLoader.isOnTarget();
        System.out.println("SetWristAngle.isFinished() - " + finished);
		return finished;
	}
	
    @Override
    protected void end() {
        System.out.println("SetWristAngle.end()");
        super.end();
    }

}
